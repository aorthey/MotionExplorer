#include "common.h"
#include "quotient_cover.h"

#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <flann/algorithms/autotuned_index.h>
 
#if OMPL_HAVE_FLANN == 0
#error FLANN is not available. Please use a different NearestNeighbors data structure.
#endif
#include <flann/flann.hpp>


#define foreach BOOST_FOREACH

using namespace ompl::geometric;

QuotientCover::QuotientCover(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QuotientCover"+std::to_string(id));
}

QuotientCover::~QuotientCover(void)
{
}

//#############################################################################
//SETUP
//#############################################################################
namespace ompl{
  template <typename _T>
  class FLANNConfigurationDistance
  {
  public:
      using ElementType = QuotientCover::Configuration *;
      using ResultType = double;

      FLANNConfigurationDistance(const typename NearestNeighbors<_T>::DistanceFunction &distFun) : distFun_(distFun)
      {
      }

      template <typename Iterator1, typename Iterator2>
      ResultType operator()(Iterator1 a, Iterator2 b, size_t /*size*/, ResultType /*worst_dist*/ = -1) const
      {
          return distFun_(*a, *b);
      }

  protected:
      const typename NearestNeighbors<_T>::DistanceFunction &distFun_;
  };

  template <typename _T, typename _Dist = FLANNConfigurationDistance<_T>>
  class NearestNeighborsFLANNConfigurationLinear : public NearestNeighborsFLANN<_T, _Dist>
  {
  public:
      NearestNeighborsFLANNConfigurationLinear()
        : NearestNeighborsFLANN<_T, _Dist>(std::shared_ptr<flann::LinearIndexParams>(new flann::LinearIndexParams()))
      {
      }
  };
  template <typename _T, typename _Dist = FLANNConfigurationDistance<_T>>
  class NearestNeighborsFLANNConfigurationHierarchicalClustering : public NearestNeighborsFLANN<_T, _Dist>
  {
  public:
      NearestNeighborsFLANNConfigurationHierarchicalClustering()
        : NearestNeighborsFLANN<_T, _Dist>(std::shared_ptr<flann::HierarchicalClusteringIndexParams>(
              new flann::HierarchicalClusteringIndexParams()))
      {
      }

  };

}


void QuotientCover::setup(void)
{
  BaseT::setup();
  if (!nearest_neighborhood){
    //return new NearestNeighborsGNAT<_T>();
    //nearest_neighborhood.reset(new NearestNeighborsGNAT<Configuration *>());
    nearest_neighborhood.reset(new NearestNeighborsGNATNoThreadSafety<Configuration *>());
    //nearest_neighborhood.reset(new NearestNeighborsSqrtApprox<Configuration *>());
    //nearest_neighborhood.reset(new NearestNeighborsLinear<Configuration *>()); 
    //nearest_neighborhood.reset(new NearestNeighborsFLANNConfigurationLinear<Configuration*>()); 
    //nearest_neighborhood.reset(new NearestNeighborsFLANNConfigurationHierarchicalClustering<Configuration*>()); 
    //nearest_neighborhood.reset(new NearestNeighborsFLANNHierarchicalClustering<Configuration *>()); 
    //nearest_neighborhood.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));

    nearest_neighborhood->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceNeighborhoodNeighborhood(a,b);
                              });
    if(!nearest_neighborhood->reportsSortedResults())
    {
      std::cout << "nearest cover does not report sorted results, but required for rewiring" << std::endl;
      exit(0);
    }

  }
  if (!nearest_vertex){
    nearest_vertex.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_vertex->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceConfigurationConfiguration(a,b);
                              });
  }

  saturated = false;
  isConnected = false;
  totalVolumeOfCover = 0.0;

  if (pdef_){
    //#########################################################################
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      OMPL_ERROR("%s: Did not specify optimization function.", getName().c_str());
      exit(0);
    }
    //#########################################################################
    checkValidity();
  }else{
    setup_ = false;
  }

}
void QuotientCover::clear()
{
  BaseT::clear();
  totalVolumeOfCover = 0.0;

  //Nearestneighbors
  if(nearest_neighborhood) nearest_neighborhood->clear();
  if(nearest_vertex) nearest_vertex->clear();

  shortest_path_start_goal.clear();
  shortest_path_start_goal_necessary_vertices.clear();

  //Cover graph
  foreach (Vertex v, boost::vertices(graph)){
    if(graph[v]!=nullptr){
      graph[v]->Remove(Q1);
    }
  }
  graph.clear();

  //PDF
  pdf_necessary_configurations.clear();
  pdf_all_configurations.clear();

  //index maps
  vertexToIndexStdMap.clear();
  indexToVertexStdMap.clear();
  index_ctr = 0;
  isConnected = false;
  saturated = false;

  if(q_start){
    q_start->Clear();
  }
  if(q_goal){
    q_goal->Clear();
  }

}
void QuotientCover::Init()
{
  graph[boost::graph_bundle].name = getName()+"_graph";
  graph[boost::graph_bundle].Q1 = Q1;
  //#########################################################################
  //Adding start configuration
  //#########################################################################
  if(const ob::State *state_start = pis_.nextStart()){
    q_start = new Configuration(Q1, state_start);
    if(!ComputeNeighborhood(q_start, true))
    {
      OMPL_ERROR("%s: Could not add start state!", getName().c_str());
      QuotientCover::Print(q_start, false);
      exit(0);
    }
  }else{
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }

  //#########################################################################
  //Adding goal configuration
  //#########################################################################
  if(const ob::State *state_goal = pis_.nextGoal()){
    q_goal = new Configuration(Q1, state_goal);
    if(!ComputeNeighborhood(q_goal, true))
    {
      OMPL_ERROR("%s: Could not add goal state!", getName().c_str());
      exit(0);
    }
  }else{
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
  //#########################################################################
  //Parent data if available
  //#########################################################################
  if(parent != nullptr)
  {
    q_start->coset = static_cast<og::QuotientCover*>(parent)->q_start;
    q_goal->coset = static_cast<og::QuotientCover*>(parent)->q_goal;
  }
  q_start->isStart = true;
  q_goal->isGoal = true;

  v_start = AddConfigurationToCover(q_start);
  //#########################################################################
  //Check saturation
  //#########################################################################
  if(q_start->GetRadius() == std::numeric_limits<double>::infinity())
  {
    OMPL_INFORM("Note: start state covers quotient-space.");
    saturated = true;
  }
  OMPL_INFORM("%s: ready with %lu states in datastructure (start radius: %f)", getName().c_str(), nearest_neighborhood->size(), q_start->GetRadius());
}

//#############################################################################
//#############################################################################
//Add Configuration to Cover methods
//#############################################################################
//#############################################################################

QuotientCover::Vertex QuotientCover::AddConfigurationToCover(Configuration *q)
{
  //###########################################################################
  //STEP1: Check that q can be projected onto a feasible area of QS
  //###########################################################################
  if(parent != nullptr){
    if(q->coset == nullptr){
      std::cout << "[WARNING] Tried adding configuration without assigned coset." << std::endl;
      QuotientCover::Print(q, false);
      exit(0);

    }
  }

  if(q->GetRadius() <= 0){
    std::cout << "[WARNING] Tried adding a zero-measure neighborhood." << std::endl;
    exit(0);
    return BGT::null_vertex();
  }

  //check if q is not already in cover
  if(IsConfigurationInsideCover(q)) return BGT::null_vertex();

  //###########################################################################
  //STEP3: Verify that neighborhood of q is not a subset of any intersecting
  //neighborhood
  //###########################################################################
  uint K = (Q1->getStateDimension()+1);

  std::vector<Configuration*> neighbors;
  nearest_neighborhood->nearestK(q, K, neighbors);

  for(uint k = 0; k < neighbors.size(); k++){
    Configuration *qn = neighbors.at(k);
    if(IsNeighborhoodInsideNeighborhood(q, qn))
      return BGT::null_vertex();
  }

  if(neighbors.empty() && !q->isStart){
    std::cout << "adding Configuration not on boundary of cover!" << std::endl;
    exit(0);
  }

  //###########################################################################
  //STEP4: Add q to cover
  //###########################################################################
  Vertex v = AddConfigurationToCoverGraph(q);

  //###########################################################################
  //STEP5: Add edge for each neighbor. Remove all subset neighbors
  //###########################################################################
  //bound number of connections. We can always rewire later on
  for(uint k = 0; k < neighbors.size(); k++){
    Configuration *qk = neighbors.at(k);
    if(!IsNeighborhoodInsideNeighborhood(qk, q))
    {
      const double dqqk = DistanceNeighborhoodNeighborhood(qk, q);
      if(dqqk < 1e-10){
        AddEdge(q, qk);
      }else{
        break;
      }
    }else{
      if(qk->isStart || qk->isGoal){
        AddEdge(q, qk);
      }else{
        RerouteEdgesFromTo(qk, q);
        RemoveConfigurationFromCover(qk);
      }
    }
  }
  return v;
}

int QuotientCover::GetNumberOfEdges(Configuration *q)
{
  if(q->index < 0) return 0;
  Vertex v = get(indexToVertex, q->index);
  return boost::out_degree(v, graph);
}

QuotientCover::Configuration* QuotientCover::SampleOnBoundaryUniformNear(Configuration *q_center, const double radius, const Configuration* q_near)
{
  Configuration *q_next = new Configuration(Q1);

  //Sample in Ambient Space
  GetQ1SamplerPtr()->sampleUniformNear(q_next->state, q_near->state /*mean*/, radius);
  ProjectConfigurationOntoBoundary(q_center, q_next);

  q_next->parent_neighbor = q_center;
  if(ComputeNeighborhood(q_next)){
    return q_next;
  }else{
    return nullptr;
  }
}

void QuotientCover::InterpolateUntilNeighborhoodBoundary(const Configuration *q_center, const Configuration *q_desired, Configuration *q_out)
{
  double radius = q_center->GetRadius();
  double distance_center_desired = DistanceConfigurationConfiguration(q_center, q_desired);
  double step = (radius + distance_center_desired)/distance_center_desired;

  Interpolate(q_center, q_desired, step, q_out);

  //############################################################################
  //DEBUG
  //############################################################################
  double d_center_outward = DistanceConfigurationConfiguration(q_center, q_out);
  if(fabs(d_center_outward - radius) > 1e-10){
    std::cout << "WARNING: interpolated point outside boundary" << std::endl;
    QuotientCover::Print(q_out, false);
    QuotientCover::Print(q_center, false);
    std::cout << "Distance: " << d_center_outward << " Radius: " << radius << std::endl;
    exit(0);
  }
}

QuotientCover::Configuration* QuotientCover::GetOutwardPointingConfiguration(Configuration *q_center)
{
  Configuration *q_inward_to_outward = new Configuration(GetQ1(), q_center->GetInwardPointingConfiguration());
  q_inward_to_outward->parent_neighbor = q_center;
  q_inward_to_outward->coset = q_center->coset;

  InterpolateUntilNeighborhoodBoundary(q_inward_to_outward, q_center, q_inward_to_outward);

  if(ComputeNeighborhood(q_inward_to_outward)){
    return q_inward_to_outward;
  }else{
    return nullptr;
  }
}

void QuotientCover::AddEdge(Configuration *q_from, Configuration *q_to)
{
  double d = DistanceQ1(q_from, q_to);
  EdgeInternalState properties(d);
  Vertex v_from = get(indexToVertex, q_from->index);
  Vertex v_to = get(indexToVertex, q_to->index);
  boost::add_edge(v_from, v_to, properties, graph);

  q_from->UpdateRiemannianCenterOfMass(Q1, q_to);
  q_to->UpdateRiemannianCenterOfMass(Q1, q_from);

  q_from->number_of_neighbors++;
  q_to->number_of_neighbors++;
}

bool QuotientCover::EdgeExists(Configuration *q_from, Configuration *q_to)
{
  Vertex v_from = get(indexToVertex, q_from->index);
  Vertex v_to = get(indexToVertex, q_to->index);
  return boost::edge(v_from, v_to, graph).second;
}

std::vector<QuotientCover::Configuration*> QuotientCover::GetIntersectingNeighborhoodConfigurations(Configuration *q)
{
  std::vector<Configuration*> neighbors;
  nearest_neighborhood->nearestR(q, 1e-10, neighbors);
  return neighbors;
}

void QuotientCover::RerouteEdgesFromTo(Configuration *q_from, Configuration *q_to)
{
  Vertex v_from = get(indexToVertex, q_from->index);
  OEIterator edge_iter, edge_iter_end, next; 
  boost::tie(edge_iter, edge_iter_end) = boost::out_edges(v_from, graph);

  for(next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
  {
    //for each edge, create a new one pointing to q_to
    ++next;
    const Vertex v_target = boost::target(*edge_iter, graph);
    Configuration *q_reroute = graph[v_target];

    AddEdge(q_reroute, q_to);
  }
}

void QuotientCover::AddConfigurationToPDF(Configuration *q)
{
  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
  q->SetPDFElement(q_element);
  if(!q->isSufficientFeasible){
    PDF_Element *q_necessary_element = pdf_necessary_configurations.add(q, q->GetRadius());
    q->SetNecessaryPDFElement(q_necessary_element);
  }
}

QuotientCover::Vertex QuotientCover::AddConfigurationToCoverGraph(Configuration *q)
{
  if(q->GetRadius()<minimum_neighborhood_radius)
  {
    OMPL_ERROR("Trying to add configuration to cover, but Neighborhood is too small.");
    std::cout << "state" << std::endl;
    Q1->printState(q->state);
    std::cout << "radius: " << q->GetRadius() << std::endl;
    exit(0);
  }
  totalVolumeOfCover += q->GetRadius();

  //###########################################################################
  //(1) add to cover graph
  //###########################################################################
  Vertex v = boost::add_vertex(q, graph);
  graph[v]->number_attempted_expansions = 0;
  graph[v]->number_successful_expansions = 0;

  //assign vertex to a unique index
  put(vertexToIndex, v, index_ctr);
  put(indexToVertex, index_ctr, v);
  graph[v]->index = index_ctr;
  index_ctr++;

  //###########################################################################
  //(2) add to nearest neighbor structure
  //###########################################################################
  nearest_neighborhood->add(q);
  nearest_vertex->add(q);

  //###########################################################################
  //(3) add to PDF
  //###########################################################################
  AddConfigurationToPDF(q);
  q->goal_distance = DistanceQ1(q, q_goal);

  return v;
}
//#############################################################################
//#############################################################################
//Remove Configuration from Cover methods
//#############################################################################
//#############################################################################
void QuotientCover::RemoveConfigurationFromCover(Configuration *q)
{
  totalVolumeOfCover -= q->GetRadius();

  //(1) Remove from PDF
  pdf_all_configurations.remove(static_cast<PDF_Element*>(q->GetPDFElement()));
  if(!q->isSufficientFeasible){
    pdf_necessary_configurations.remove(static_cast<PDF_Element*>(q->GetNecessaryPDFElement()));
  }


  //(2) Remove from nearest neighbors structure
  nearest_neighborhood->remove(q);
  nearest_vertex->remove(q);

  //erase entry from indexmap
  Vertex vq = get(indexToVertex, q->index);
  vertexToIndexStdMap.erase(vq);
  indexToVertexStdMap.erase(q->index);

  //check that they do not exist in indextovertex anymore
  if(indexToVertexStdMap.count(q->index) > 0){
    vertex_index_type vit_before = get(vertexToIndex, vq);
    vertex_index_type vit_after = get(vertexToIndex, vq);
    std::cout << vit_before << " - " << vit_after << std::endl;
    exit(0);
  }

  boost::clear_vertex(vq, graph);
  boost::remove_vertex(vq, graph);
  //(3) Remove from cover graph
  q->Remove(Q1);
  delete q;
  q=nullptr;
}

bool QuotientCover::ComputeNeighborhood(Configuration *q, bool verbose)
{
  if(q==nullptr) return false;
  if(IsConfigurationInsideCover(q)){
    if(verbose) std::cout << "[ComputeNeighborhood] State Rejected: Inside Cover" << std::endl;
    q->Remove(Q1);
    q=nullptr;
    return false;
  }

  // QuotientCover::Print(q,false);
  // Q1->printState(q->state);
  bool feasible = Q1->isValid(q->state);

  if(feasible){

    if(IsOuterRobotFeasible(q->state))
    {
      q->isSufficientFeasible = true;
      q->SetOuterRadius(DistanceOuterRobotToObstacle(q->state));
    }

    q->SetRadius(DistanceInnerRobotToObstacle(q->state));
    if(q->GetRadius()<=minimum_neighborhood_radius){
      if(verbose) std::cout << "[ComputeNeighborhood] State Rejected: Radius Too Small (radius="<< q->GetRadius() << ")" << std::endl;
      q->Remove(Q1);
      q=nullptr;
      return false;
    }
  }else{
    if(verbose) std::cout << "[ComputeNeighborhood] State Rejected: Infeasible" << std::endl;
    q->Remove(Q1);
    q=nullptr;
    return false;
  }
  GetCosetFromQuotientSpace(q);

  return true;
}

bool QuotientCover::IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn)
{
  return (DistanceConfigurationNeighborhood(q, qn) <= 1e-10);
}

bool QuotientCover::IsConfigurationInsideCover(Configuration *q)
{
  //std::cout << "is inside cover" << std::endl;
  std::vector<Configuration*> neighbors;
  nearest_neighborhood->nearestK(q, 2, neighbors);
  //std::cout << "nbs: " << neighbors.size() << std::endl;
  if(neighbors.size()<=1) return false;
  return IsConfigurationInsideNeighborhood(q, neighbors.at(0)) && IsConfigurationInsideNeighborhood(q, neighbors.at(1));
}

double QuotientCover::GetImportance() const
{
  //approximation of total volume covered
  //double percentageCovered = totalVolumeOfCover/Q1->getStateSpace()->getMeasure();
  //the more sampled, the less important this space becomes
  //return 1.0/(totalVolumeOfCover+1);

  return 1.0/(boost::num_vertices(graph)+1);
  //double importance = pow((double)2.0, level);
  //return importance;
}

void QuotientCover::GetCosetFromQuotientSpace(Configuration *q)
{
  if(parent != nullptr)
  {
    //(1) project q onto Q0, obtain q_projected
    //(2) find nearest configuration to q_projected on Q0
    //(3) set nearest to coset of q

    og::QuotientCover *qcc_parent = static_cast<og::QuotientCover*>(parent);
    Configuration *q_projected = new Configuration(Q0);
    ExtractQ0Subspace(q->state, q_projected->state);
    q->coset = qcc_parent->Nearest(q_projected);
    q_projected->Remove(Q0);
    // Print(q);
    // std::cout << "coset: " << q->coset->index << std::endl;
    delete q_projected;

  }
}


//#############################################################################
//Sampling Configurations (on quotient space)
//#############################################################################

void QuotientCover::SampleGoal(Configuration *q)
{
  q->state = Q1->cloneState(q_goal->state);
  q->coset = q_goal->coset;
}

void QuotientCover::SampleUniform(Configuration *q)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q->state);
  }else{
    ob::State *stateX1 = X1->allocState();
    ob::State *stateQ0 = Q0->allocState();
    X1_sampler->sampleUniform(stateX1);
    q->coset = static_cast<og::QuotientCover*>(parent)->SampleUniformQuotientCover(stateQ0);
    if(q->coset == nullptr){
      OMPL_ERROR("no coset found for state");
      Q1->printState(q->state);
      exit(0);
    }
    MergeStates(stateQ0, stateX1, q->state);
    X1->freeState(stateX1);
    Q0->freeState(stateQ0);
  }
}
void QuotientCover::SampleRandomNeighborhoodBoundary(Configuration *q)
{
  Configuration *q_nearest = pdf_all_configurations.sample(rng_.uniform01());
  q_nearest->number_attempted_expansions++;
  pdf_all_configurations.update(static_cast<PDF_Element*>(q_nearest->GetPDFElement()), q_nearest->GetImportance());

  SampleNeighborhoodBoundaryHalfBall(q, q_nearest);
  q->parent_neighbor = q_nearest;
  GetCosetFromQuotientSpace(q);
}

//#############################################################################
//Sampling Configurations (On boundary of cover)
//#############################################################################

QuotientCover::Configuration* QuotientCover::SampleCoverBoundary(std::string type)
{
  Configuration *q_random = new Configuration(Q1);
  if(type == "voronoi"){
    SampleUniform(q_random);
  }else if(type == "goal"){
    SampleGoal(q_random);
  }else if(type == "boundary"){
    SampleRandomNeighborhoodBoundary(q_random);
    return q_random;
  }else{
    std::cout << "sampling type " << type << " not recognized." << std::endl;
    exit(0);
  }

  Configuration *q_nearest = Nearest(q_random);
  Connect(q_nearest, q_random, q_random);
  q_random->parent_neighbor = q_nearest;
  if(q_random == q_nearest)
  {
    std::cout << "sampling invalid: nearest and random are equal" << std::endl;
    exit(0);
  }
  GetCosetFromQuotientSpace(q_random);
  return q_random;
}

//#############################################################################
// Main Sample Function
//#############################################################################

QuotientCover::Configuration* QuotientCover::Sample()
{
  return SampleCoverBoundary();
}
QuotientCover::Configuration* QuotientCover::SampleCoverBoundary()
{
  Configuration *q_random = SampleCoverBoundary("boundary");
  return q_random;
}

QuotientCover::Configuration* QuotientCover::SampleCoverBoundaryValid(ob::PlannerTerminationCondition &ptc)
{
  Configuration *q = nullptr;
  while(!ptc){
    q = SampleCoverBoundary();
    //ignore samples inside cover (rejection sampling)
    if(ComputeNeighborhood(q)) break;
    else q = nullptr;
  }
  return q;
}

QuotientCover::Configuration* QuotientCover::SampleUniformQuotientCover(ob::State *state) 
{
  Configuration *q_coset = pdf_all_configurations.sample(rng_.uniform01());

  Q1_sampler->sampleUniformNear(state, q_coset->state, q_coset->GetRadius());
  //if(q_coset->isSufficientFeasible)
  //{
  //  //project onto a random shell outside of sufficient neighborhood
  //  double r_necessary = q_coset->GetRadius();
  //  double r_sufficient = q_coset->openNeighborhoodOuterRadius;
  //  double r_01 = rng_.uniform01();
  //  double r_shell = r_sufficient + r_01*(r_necessary - r_sufficient);
  //  double d = Q1->distance(q_coset->state, state);
  //  Q1->getStateSpace()->interpolate(q_coset->state, state, r_shell/d, state);
  //}

  //Project sampled state onto the boundary!
  double d = Q1->distance(q_coset->state, state);
  Q1->getStateSpace()->interpolate(state, q_coset->state, q_coset->GetRadius()/d, q_coset->state);

  return q_coset;
}

bool QuotientCover::SampleNeighborhoodBoundary(Configuration *q_random, const Configuration *q_center)
{
  //sample on boundary of open neighborhood
  // (1) first sample gaussian point qk around q
  // (2) project qk onto neighborhood
  // (*) this works because gaussian is symmetric around origin
  // (*) this does not work when qk is near to q, so we need to sample as long
  // as it is not near
  //
  double radius = q_center->GetRadius();
  double dist_q_qk = 0;

  if(minimum_neighborhood_radius >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "Configuration " << std::endl;
    Print(q_center);
    std::cout << "radius: " << radius << std::endl;
    std::cout << "min distance: " << minimum_neighborhood_radius << std::endl;
    std::cout << std::string(80, '#') << std::endl;
    exit(0);
  }
  //sample as long as we are inside the ball of radius
  //minimum_neighborhood_radius
  while(dist_q_qk <= minimum_neighborhood_radius){
    Q1_sampler->sampleGaussian(q_random->state, q_center->state, 1);
    dist_q_qk = Q1->distance(q_center->state, q_random->state);
  }
  Q1->getStateSpace()->interpolate(q_center->state, q_random->state, radius/dist_q_qk, q_random->state);

  return true;
}
bool QuotientCover::SampleNeighborhoodBoundaryHalfBall(Configuration *q_random, const Configuration *q_center)
{
  SampleNeighborhoodBoundary(q_random, q_center);

  //q0 ---- q1 (q_center) ----- q2 (q_random)
  Configuration *q0 = q_center->parent_neighbor;

  //case1: no parent is available, there is no halfball
  if(q0==nullptr) return true;

  //case2: q_center is overlapping q0
  double radius_q0 = q0->GetRadius();
  double radius_q1 = q_center->GetRadius();
  if(2*radius_q0 < radius_q1) return true;

  //case3: q_random is lying already on outer half ball
  double d12 = DistanceConfigurationConfiguration(q_center, q_random);
  double d02 = DistanceConfigurationConfiguration(q0, q_random);
  if(d02 > d12) return true;

  //case4: q_random is lying on inner half ball => point reflection in q_center.
  //this is accomplished by interpolating from q_random to q_center, and then
  //following the same distance until the outer ball is hit
  Q1->getStateSpace()->interpolate(q_random->state, q_center->state, 2, q_random->state);

  return true;
}

//@brief: move from q_from to q_to until the neighborhood of q_from is
//intersected. Return the intersection point in q_out
void QuotientCover::Connect(const Configuration *q_from, const Configuration *q_to, Configuration *q_out)
{
  double radius = q_from->GetRadius();
  double dist_qfrom_qto = DistanceQ1(q_from, q_to);
  Q1->getStateSpace()->interpolate(q_from->state, q_to->state, radius/dist_qfrom_qto, q_out->state);

  double dist_qfrom_qout = DistanceQ1(q_from, q_out);
  if(fabs(radius - dist_qfrom_qout) > 1e-10)
  {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "from:" << std::endl;
    Print(q_from, false);
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "to:" << std::endl;
    Print(q_to, false);
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "out:" << std::endl;
    Print(q_out, false);
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Radius q_from            : " << radius << std::endl;
    std::cout << "Distance q_from to q_out : " << dist_qfrom_qout << std::endl;
    exit(-1);
  }

  //if(parent==nullptr){
  //  Q1->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
  //}else{
  //  std::vector<const Configuration*> path = GetInterpolationPath(q_from, q_to);
  //  const Configuration *q_next = nullptr;
  //  double d = 0;
  //  double d_last_to_next = 0;
  //  uint ctr = 0;
  //  while(d < step_size && ctr < path.size()){
  //    d_last_to_next = DistanceQ1(path.at(ctr), path.at(ctr+1));
  //    d += d_last_to_next;
  //    q_next = path.at(ctr+1);
  //    ctr++;
  //  }
  //  const Configuration *q_last = path.at(ctr-1);
  //  double step = d_last_to_next - (d-step_size);
  //  Q1->getStateSpace()->interpolate(q_last->state, q_next->state, step/d_last_to_next, q_interp->state);
  //}

  // if(parent == nullptr)
  // {
  //   double dist_qfrom_qto = DistanceConfigurationConfiguration(q_from, q_to);
  //   Q1->getStateSpace()->interpolate(q_from->state, q_to->state, radius/dist_qfrom_qto, q_out->state);
  // }else{
  //   std::cout << "Connect() NYI" << std::endl;
  //   exit(0);

  // }

}

bool QuotientCover::IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs)
{
  double distance_centers = DistanceQ1(lhs, rhs);
  double radius_rhs = rhs->GetRadius();
  double radius_lhs = lhs->GetRadius();
  if(radius_rhs < minimum_neighborhood_radius || radius_lhs < minimum_neighborhood_radius)
  {
    std::cout << "neighborhood inclusion failed" << std::endl;
    std::cout << "neighborhood1 with radius: " << radius_rhs << std::endl;
    Print(rhs);
    std::cout << "neighborhood2 with radius: " << radius_lhs << std::endl;
    Print(lhs);
    exit(0);
  }
  return (radius_rhs > (radius_lhs + distance_centers));
}

std::vector<QuotientCover::Configuration*> QuotientCover::GetConfigurationsInsideNeighborhood(Configuration *q)
{
  std::vector<Configuration*> neighbors;
  nearest_vertex->nearestR(q, q->GetRadius(), neighbors);
  return neighbors;
}

void QuotientCover::RewireConfiguration(Configuration *q)
{
  std::vector<Configuration*> neighbors;
  Vertex v = get(indexToVertex, q->index);
  uint K = boost::degree(v, graph)+1;

  nearest_neighborhood->nearestK(q, K, neighbors);
  Configuration *qn = neighbors.at(K-1);
  double dn = DistanceNeighborhoodNeighborhood(q, qn);
  if(dn <= 1e-10){
    AddEdge(q, qn);
  }
}

void QuotientCover::CheckConfigurationIsOnBoundary(Configuration *q_boundary, Configuration *q)
{
  double d_outcome = DistanceConfigurationConfiguration(q_boundary, q);
  if(fabs(d_outcome - q->GetRadius())>1e-10)
  {
      std::cout << "Point not on boundary!" << std::endl;
      QuotientCover::Print(q, false);
      QuotientCover::Print(q_boundary, false);
      std::cout << "radius neighborhood: " << q->GetRadius() << std::endl;
      std::cout << "dist to point: " << d_outcome << std::endl;
      exit(1);
  }
}

QuotientCover::Configuration* QuotientCover::Nearest(Configuration *q) const
{
  return nearest_neighborhood->nearest(q);
}

//@TODO: should be fixed to reflect the distance on the underlying
//quotient-space, i.e. similar to QMPConnect. For now we use DistanceQ1

bool QuotientCover::Interpolate(const Configuration *q_from, Configuration *q_to)
{
  return Interpolate(q_from, q_to, q_to);
}

bool QuotientCover::Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration *q_interp)
{
  double d = DistanceConfigurationConfiguration(q_from, q_to);
  double radius = q_from->GetRadius();
  double step_size = radius/d;
  bool success = Interpolate(q_from, q_to, step_size, q_interp);
  return success;
}
//stepsize \in [0,1]
bool QuotientCover::Interpolate(const Configuration *q_from, const Configuration *q_to, double step_size, Configuration *q_interp)
{
  if(parent==nullptr){
    Q1->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
  }else{
    //move along path until step_size is reached. Then interpolate between the
    //waypoints to get the point on the path of distance step_size from q_from

    //q_from ---- q1 ----- q2 --- ... ---- q_to
    //-----------------------d----------------
    //--------------d_step

    //@TODO: optimize that we do not use distance config config two times, or
    //use internal cache?
    if(q_to->coset == nullptr)
    {
      if(q_from->coset == nullptr) {
        Q1->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
      }else{
        double d_from_to = DistanceConfigurationConfiguration(q_from, q_to);
        double radius = q_from->GetRadius();
        Q1->getStateSpace()->interpolate(q_from->state, q_to->state, radius/d_from_to, q_interp->state);
      }
    }else{
      double d_from_to = DistanceConfigurationConfiguration(q_from, q_to);
      double d_step = d_from_to*step_size;

      std::vector<const Configuration*> path = GetInterpolationPath(q_from, q_to);
      double d = 0;
      double d_last_to_next = 0;
      uint ctr = 0;

      while(d < d_step && ctr < path.size()-1){
        d_last_to_next = DistanceQ1(path.at(ctr), path.at(ctr+1));
        d += d_last_to_next;
        ctr++;
      }

      const Configuration *q_next = path.at(ctr);
      const Configuration *q_last = path.at(ctr-1);
      double step = d_last_to_next - (d - d_step);
      Q1->getStateSpace()->interpolate(q_last->state, q_next->state, step/d_last_to_next, q_interp->state);
      // std::cout << "radius q_from: " << q_from->GetRadius() << std::endl;
      // std::cout << "dist last next: " << d_last_to_next << std::endl;
      // std::cout << "step: " << step << std::endl;
    }
  }

  return true;
}

bool QuotientCover::InterpolateOnBoundary(const Configuration* q_center, const Configuration* q1, const Configuration* q2, double step, Configuration* q_out)
{
  Q1->getStateSpace()->interpolate(q1->state, q2->state, step, q_out->state);
  ProjectConfigurationOntoBoundary(q_center, q_out);
  return true;
}
void QuotientCover::ProjectConfigurationOntoBoundary(const Configuration *q_center, Configuration* q_projected)
{
  const double d_center_to_proj = DistanceConfigurationConfiguration(q_center, q_projected);
  double step_size = q_center->GetRadius()/d_center_to_proj;
  Interpolate(q_center, q_projected, step_size, q_projected);
}
QuotientCover::Configuration* QuotientCover::NearestConfigurationOnBoundary(Configuration *q_center, const Configuration* q_outside)
{
  Configuration *q_projected = new Configuration(Q1);
  const double d_center_to_proj = DistanceConfigurationConfiguration(q_center, q_outside);
  double step_size = q_center->GetRadius()/d_center_to_proj;
  Q1->getStateSpace()->interpolate(q_center->state, q_outside->state, step_size, q_projected->state);
  q_projected->parent_neighbor = q_center;
  if(ComputeNeighborhood(q_projected)){
    return q_projected;
  }else{
    return nullptr;
  }
}

//#############################################################################
//Distance Functions
//#############################################################################

double QuotientCover::DistanceQ1(const Configuration *q_from, const Configuration *q_to)
{
  return Q1->distance(q_from->state, q_to->state);
}

double QuotientCover::DistanceX1(const Configuration *q_from, const Configuration *q_to)
{
  ob::State *stateFrom = X1->allocState();
  ob::State *stateTo = X1->allocState();
  ExtractX1Subspace(q_from->state, stateFrom);
  ExtractX1Subspace(q_to->state, stateTo);
  double d = X1->distance(stateFrom, stateTo);
  X1->freeState(stateFrom);
  X1->freeState(stateTo);
  return d;
}

double QuotientCover::DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to)
{
  if(parent == nullptr){
    //the very first quotient-space => usual metric
    return DistanceQ1(q_from, q_to);
  }else{
    if(q_to->coset == nullptr || q_from->coset == nullptr)
    {
      //std::cout << std::string(80, '#') << std::endl;
      //std::cout << "[ERROR] could not find coset for a configuration" << std::endl;
      //std::cout << std::string(80, '#') << std::endl;
      //std::cout << "from:"; Print(q_from, false);
      //std::cout << std::string(80, '-') << std::endl;
      //std::cout << "to:"; Print(q_to);
      //std::cout << std::string(80, '-') << std::endl;
      //std::cout << *this << std::endl;
      //exit(0);
      //could not find a coset on the quotient-space  => usual metric
      //fall-back metric
      return DistanceQ1(q_from, q_to);
    }
    return DistanceConfigurationConfigurationCover(q_from, q_to)+DistanceX1(q_from, q_to);
  }
}


double QuotientCover::DistanceConfigurationConfigurationCover(const Configuration *q_from, const Configuration *q_to)
{
  std::vector<const Configuration*> path = GetInterpolationPath(q_from, q_to);
  double d = 0;
  for(uint k = 0; k < path.size()-1; k++){
    d += DistanceQ1(path.at(k), path.at(k+1));
  }
  return d;
}

std::vector<const QuotientCover::Configuration*> QuotientCover::GetInterpolationPath(const Configuration *q_from, const Configuration *q_to)
{
  if(q_from->coset == nullptr || q_to->coset == nullptr){
    OMPL_ERROR("Cannot interpolate without cosets");
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "[ERROR] could not find coset for a configuration" << std::endl;
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "from:"; Print(q_from, false);
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "to:"; Print(q_to);
    std::cout << std::string(80, '-') << std::endl;
    std::cout << *this << std::endl;
    throw "InterpolateWithoutCosets";
    exit(1);
  }
  std::vector<const Configuration*> path_Q1;

  og::QuotientCover *parent_chart = dynamic_cast<og::QuotientCover*>(parent);

  //(0) get shortest path on Q0 between q_from->coset to q_to->coset
  std::vector<const Configuration*> path_Q0_cover = parent_chart->GetCoverPath(q_from->coset, q_to->coset);

  if(path_Q0_cover.empty()){
    //(1) cosets are equivalent => straight line interpolation
    path_Q1.push_back(q_from);
    path_Q1.push_back(q_to);
  }else{
    path_Q0_cover.erase(path_Q0_cover.begin());
    path_Q0_cover.pop_back();

    if(path_Q0_cover.empty()){
      //(2) cosets are neighbors => straight line interpolation
      path_Q1.push_back(q_from);
      path_Q1.push_back(q_to);
    }else{
      //(3) cosets have nonzero path between them => interpolate along that path
      ob::State *s_fromQ0 = Q0->allocState();
      ExtractQ0Subspace(q_from->state, s_fromQ0);
      Configuration *q_fromQ0 = new Configuration(Q0, s_fromQ0);

      ob::State *s_toQ0 = Q0->allocState();
      ExtractQ0Subspace(q_to->state, s_toQ0);
      Configuration *q_toQ0 = new Configuration(Q0, s_toQ0);

      //distance along cover path
      double d = 0;
      std::vector<double> d_vec;
      double d0 = parent_chart->DistanceConfigurationConfiguration(q_fromQ0, path_Q0_cover.at(0));
      d_vec.push_back(d0); d+=d0;
      for(uint k = 1; k < path_Q0_cover.size(); k++){
        double dk = parent_chart->DistanceConfigurationConfiguration(path_Q0_cover.at(k-1), path_Q0_cover.at(k));
        d_vec.push_back(dk); d+=dk;
      }
      double d1= parent_chart->DistanceConfigurationConfiguration(path_Q0_cover.back(), q_toQ0);
      d_vec.push_back(d1); d+=d1;

      q_toQ0->Remove(Q0);
      q_fromQ0->Remove(Q0);
      Q0->freeState(s_toQ0);
      Q0->freeState(s_fromQ0);

      //interpolate on X1
      ob::State *s_fromX1 = X1->allocState();
      ExtractX1Subspace(q_from->state, s_fromX1);
      ob::State *s_toX1 = X1->allocState();
      ExtractX1Subspace(q_to->state, s_toX1);

      double d_next = 0;
      path_Q1.push_back(q_from);
      for(uint k = 0; k < path_Q0_cover.size(); k++){
        ob::State *s_kX1 = X1->allocState();
        d_next += d_vec.at(k);
        X1->getStateSpace()->interpolate(s_fromX1, s_toX1, d_next/d, s_kX1);

        Configuration *qk = new Configuration(Q1);
        MergeStates(path_Q0_cover.at(k)->state, s_kX1, qk->state);
        path_Q1.push_back(qk);
        X1->freeState(s_kX1);
      }
      path_Q1.push_back(q_to);

      X1->freeState(s_toX1);
      X1->freeState(s_fromX1);
    }
  }
  return path_Q1;
}

double QuotientCover::DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_to = q_to->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_to);
  return std::max(d - d_to, 0.0);
}
double QuotientCover::DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_from = q_from->GetRadius();
  double d_to = q_to->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_to);

  if(d!=d){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << *this << std::endl;
    std::cout << "at " << getName() << ":" << id << std::endl;
    std::cout << "NaN detected." << std::endl;
    std::cout << "d_from " << d_from << std::endl;
    std::cout << "d_to " << d_to << std::endl;
    std::cout << "d " << d << std::endl;
    std::cout << "configuration 1: " << std::endl;
    Print(q_from, false);
    std::cout << "configuration 2: " << std::endl;
    Print(q_to, false);
    std::cout << std::string(80, '-') << std::endl;
    throw "";
    exit(1);
  }
  double d_open_neighborhood_distance = (double)std::max(d - d_from - d_to, 0.0); 
  //std::cout << d << "," << d_from << "," << d_to << "," << d_open_neighborhood_distance << std::endl;
  return d_open_neighborhood_distance;
}

bool QuotientCover::GetSolution(ob::PathPtr &solution)
{
  if(!isConnected){
    Configuration* qn = Nearest(q_goal);
    double d_goal = DistanceNeighborhoodNeighborhood(qn, q_goal);
    if(d_goal < 1e-10){
      q_goal->parent_neighbor = qn;
      v_goal = AddConfigurationToCover(q_goal);
      isConnected = true;
    }
  }
  if(isConnected){
    auto gpath(std::make_shared<PathGeometric>(Q1));
    shortest_path_start_goal.clear();
    shortest_path_start_goal_necessary_vertices.clear();
    shortest_path_start_goal = GetCoverPath(v_start, v_goal);
    gpath->clear();
    for(uint k = 0; k < shortest_path_start_goal.size(); k++){
      Configuration *q = graph[shortest_path_start_goal.at(k)];
      //std::cout << "vertex " << q->index << " " << (q->isSufficientFeasible?"sufficient":"") << std::endl;
      gpath->append(q->state);
      if(!q->isSufficientFeasible)
      {
        shortest_path_start_goal_necessary_vertices.push_back(shortest_path_start_goal.at(k));
      }
    }
    solution = gpath;
    return true;
  }
  return false;
}

const QuotientCover::Graph& QuotientCover::GetGraph() const
{
  return graph;
}

QuotientCover::Configuration* QuotientCover::GetStartConfiguration() const
{
  return q_start;
}
QuotientCover::Configuration* QuotientCover::GetGoalConfiguration() const
{
  return q_goal;
}

const QuotientCover::PDF& QuotientCover::GetPDFNecessaryConfigurations() const
{
  return pdf_necessary_configurations;
}

const QuotientCover::PDF& QuotientCover::GetPDFAllConfigurations() const
{
  return pdf_all_configurations;
}
const QuotientCover::NearestNeighborsPtr& QuotientCover::GetNearestNeighborsCover() const
{
  return nearest_neighborhood;
}
const QuotientCover::NearestNeighborsPtr& QuotientCover::GetNearestNeighborsVertex() const
{
  return nearest_vertex;
}

struct found_goal {}; // exception for termination

template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};

std::vector<const QuotientCover::Configuration*> QuotientCover::GetCoverPath(const Configuration *q_source, const Configuration *q_sink)
{
  const Vertex v_source = get(indexToVertex, q_source->index);
  const Vertex v_sink = get(indexToVertex, q_sink->index);
  std::vector<const Configuration*> q_path;
  if(v_source == v_sink) return q_path;
  std::vector<Vertex> v_path = GetCoverPath(v_source, v_sink);
  for(uint k = 0; k < v_path.size(); k++){
    q_path.push_back(graph[v_path.at(k)]);
  }
  return q_path;
}
std::vector<QuotientCover::Vertex> QuotientCover::GetCoverPath(const Vertex& v_source, const Vertex& v_sink)
{
  std::vector<Vertex> path;
  std::vector<Vertex> prev(boost::num_vertices(graph));
  auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost), get(boost::edge_bundle, graph));
  auto predecessor = boost::make_iterator_property_map(prev.begin(), vertexToIndex);

  //check that vertices exists in graph
  if(verbose>3) std::cout << std::string(80, '-') << std::endl;
  if(verbose>3) std::cout << "searching from " << get(vertexToIndex, v_source) << " to " << get(vertexToIndex, v_sink) << std::endl;
  if(verbose>3) Q1->printState(graph[v_source]->state);
  if(verbose>3) Q1->printState(graph[v_sink]->state);
  if(verbose>3) std::cout << graph << std::endl;

  try{
    boost::astar_search(graph, v_source,
                    [this, v_sink](const Vertex &v)
                    {
                        return ob::Cost(DistanceQ1(graph[v], graph[v_sink]));
                    },
                      predecessor_map(predecessor)
                      .weight_map(weight)
                      .visitor(astar_goal_visitor<Vertex>(v_sink))
                      .vertex_index_map(vertexToIndex)
                      .distance_compare([this](EdgeInternalState c1, EdgeInternalState c2)
                                        {
                                            return opt_->isCostBetterThan(c1.getCost(), c2.getCost());
                                        })
                      .distance_combine([this](EdgeInternalState c1, EdgeInternalState c2)
                                        {
                                            return opt_->combineCosts(c1.getCost(), c2.getCost());
                                        })
                      .distance_inf(opt_->infiniteCost())
                      .distance_zero(opt_->identityCost())
                    );
  }catch(found_goal fg){
    for(Vertex v = v_sink;; v = prev[get(vertexToIndex, v)])
    {
      path.push_back(v);
      if(verbose>3)std::cout << std::string(80, '-') << std::endl;
      if(verbose>3)std::cout << "v:" << graph[v]->index << std::endl;
      if(verbose>3)std::cout << "idx:" << get(vertexToIndex, v) << std::endl;
      if(graph[prev[get(vertexToIndex, v)]]->index == graph[v]->index)
        break;
    }
    std::reverse(path.begin(), path.end());
  }
  return path;
}


PlannerDataVertexAnnotated QuotientCover::getAnnotatedVertex(ob::State* state, double radius, bool sufficient) const
{
  PlannerDataVertexAnnotated pvertex(state);
  pvertex.SetLevel(GetLevel());
  pvertex.SetOpenNeighborhoodDistance(radius);

  if(!state){
    std::cout << "vertex state does not exists" << std::endl;
    std::cout << *this << std::endl;
    Q1->printState(state);
    exit(0);
  }

  using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;
  if(sufficient){
    pvertex.SetFeasibility(FeasibilityType::SUFFICIENT_FEASIBLE);
  }else{
    pvertex.SetFeasibility(FeasibilityType::FEASIBLE);
  }
  return pvertex;
}
PlannerDataVertexAnnotated QuotientCover::getAnnotatedVertex(Vertex vertex) const
{
  ob::State *state = graph[vertex]->state;
  return getAnnotatedVertex(state, graph[vertex]->GetRadius(), graph[vertex]->isSufficientFeasible);
}


void QuotientCover::getPlannerData(base::PlannerData &data) const
{
  if(verbose>0) std::cout << "graph has " << boost::num_vertices(graph) << " (idx: " << index_ctr << ") vertices and " << boost::num_edges(graph) << " edges." << std::endl;
  std::map<const uint, const ob::State*> indexToStates;

  PlannerDataVertexAnnotated pstart = getAnnotatedVertex(v_start);
  indexToStates[graph[v_start]->index] = pstart.getState();
  data.addStartVertex(pstart);

  if(isConnected){
    PlannerDataVertexAnnotated pgoal = getAnnotatedVertex(v_goal);
    indexToStates[graph[v_goal]->index] = pgoal.getState();
    data.addGoalVertex(pgoal);
  }

  foreach( const Vertex v, boost::vertices(graph))
  {
    if(indexToStates.find(graph[v]->index) == indexToStates.end()) {
      PlannerDataVertexAnnotated p = getAnnotatedVertex(v);
      indexToStates[graph[v]->index] = p.getState();
      data.addVertex(p);
    }
    //otherwise vertex is a goal or start vertex and has already been added
  }
  foreach (const Edge e, boost::edges(graph))
  {
    const Vertex v1 = boost::source(e, graph);
    const Vertex v2 = boost::target(e, graph);

    const ob::State *s1 = indexToStates[graph[v1]->index];
    const ob::State *s2 = indexToStates[graph[v2]->index];
    PlannerDataVertexAnnotated p1(s1);
    PlannerDataVertexAnnotated p2(s2);
    data.addEdge(p1,p2);
  }
  if(verbose>0) std::cout << "added " << data.numVertices() << " vertices and " << data.numEdges() << " edges."<< std::endl;
}

void QuotientCover::Print(const Configuration *q, bool stopOnError) const
{
  Q1->printState(q->state);
  std::cout << " | index: " << q->index;
  std::cout << " | radius: " << q->GetRadius();
  std::cout << " | distance goal: " << q->GetGoalDistance();
  std::cout << " | coset : " << (q->coset==nullptr?"-":std::to_string(q->coset->index)) << std::endl;
  std::cout << " | parent index : " << (q->parent_neighbor==nullptr?"-":std::to_string(q->parent_neighbor->index)) << std::endl;
  std::cout << " | neighbors : " << (q->number_of_neighbors) << std::endl;
  if(q->index < 0)
  {
    std::cout << "[### STATE NOT MEMBER OF COVER]" << std::endl;
  }
  if(parent != nullptr && q->coset==nullptr)
  {
    std::cout << "### STATE HAS NO COSET" << std::endl;
    if(stopOnError) exit(0);
  }
  if(q->GetRadius() < minimum_neighborhood_radius)
  {
    std::cout << "### STATE HAS ZERO-MEASURE NEIGHBORHOOD" << std::endl;
    std::cout << "### -- RADIUS OF NEIGHBORHOOD: " << q->GetRadius() << " (minimum: " << minimum_neighborhood_radius << ")" << std::endl;
    if(stopOnError) exit(0);
  }
}
void QuotientCover::Print(std::ostream& out) const
{
  BaseT::Print(out);

  out << std::endl << " |---- [Cover] has " << boost::num_vertices(graph) << " vertices and " << boost::num_edges(graph) << " edges. ";
    //<< num_components << " connected components.";
}

namespace ompl{
  namespace geometric{
    std::ostream& operator<< (std::ostream& out, const QuotientCover::Graph& graph)
    {
      out << std::string(80, '-') << std::endl;
      out << "[Graph]" << std::endl;
      out << std::string(80, '-') << std::endl;
      ob::SpaceInformationPtr Q1 = graph[boost::graph_bundle].Q1;

      foreach(const QuotientCover::Vertex v, boost::vertices(graph))
      {
        QuotientCover::Configuration *q = graph[v];
        std::cout << "vertex " << q->index << " radius " << q->GetRadius() << " : ";
        Q1->printState(q->state);
      }
      foreach (const QuotientCover::Edge e, boost::edges(graph))
      {
        const QuotientCover::Vertex v1 = boost::source(e, graph);
        const QuotientCover::Vertex v2 = boost::target(e, graph);
        std::cout << "edge from " << graph[v1]->index << " to " << graph[v2]->index << " (weight " << graph[e].getWeight() << ")" << std::endl;
      }

      out << std::string(80, '-') << std::endl;
      out << "--- graph " << graph[boost::graph_bundle].name << std::endl;
      out << "---       has " << boost::num_vertices(graph) << " vertices and " << boost::num_edges(graph) << " edges." << std::endl;
      out << std::string(80, '-') << std::endl;
      return out;
    }
  }
}
