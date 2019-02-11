#include "common.h"
#include "quotient_cover.h"
#include "metric/quotient_metric.h"
#include "metric/quotient_metric_shortest_path.h"

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
const QuotientMetricPtr& QuotientCover::GetMetric()
{
  return metric;
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

void QuotientCover::SetMetric(const std::string& s_metric)
{
  if(metric!=nullptr){
    std::cout << "Tried to set metric more than once " << s_metric << std::endl;
    exit(0);
  }
  if(s_metric == "euclidean"){
    metric = std::make_shared<QuotientMetric>(this);
  }else if(s_metric == "shortestpath"){
    metric = std::make_shared<QuotientMetricShortestPath>(this);
  }else{
    std::cout << "Metric is not known: " << s_metric << std::endl;
    exit(0);
  }
}

void QuotientCover::setup(void)
{
  if(metric==nullptr){
    SetMetric("euclidean");
  }
  BaseT::setup();
  if (!nearest_neighborhood){
    //return new NearestNeighborsGNAT<_T>();
    //nearest_neighborhood.reset(new NearestNeighborsGNAT<Configuration *>());
    //nearest_neighborhood.reset(new NearestNeighborsGNATNoThreadSafety<Configuration *>());
    //nearest_neighborhood.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    //nearest_neighborhood.reset(new NearestNeighborsSqrtApprox<Configuration *>());
    //nearest_neighborhood.reset(new NearestNeighborsLinear<Configuration *>()); 
    nearest_neighborhood.reset(new NearestNeighborsFLANNConfigurationLinear<Configuration*>()); 
    //nearest_neighborhood.reset(new NearestNeighborsFLANNConfigurationHierarchicalClustering<Configuration*>()); 
    //nearest_neighborhood.reset(new NearestNeighborsFLANNHierarchicalClustering<Configuration *>()); 
    //nearest_neighborhood.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));

    nearest_neighborhood->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return metric->DistanceNeighborhoodNeighborhood(a,b);
                              });
    if(!nearest_neighborhood->reportsSortedResults())
    {
      std::cout << "nearest cover does not report sorted results, but required for rewiring" << std::endl;
      exit(0);
    }

  }
  if (!nearest_configuration){
    nearest_configuration.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_configuration->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return metric->DistanceConfigurationConfiguration(a,b);
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
    checkValidity();
    //Test();
    setup_ = true;
  }else{
    setup_ = false;
  }

}
void QuotientCover::Test()
{
  //#########################################################################
  //TEST0: Add/Delete mockup start element
  //#########################################################################
  Configuration *q = new Configuration(Q1);
  q->openNeighborhoodRadius = 0.5;
  q->isStart = true;
  AddConfigurationToCover(q);
  RemoveConfigurationFromCover(q);
  //#########################################################################
  //TEST1: Delete last element
  //#########################################################################
  Configuration *q1 = new Configuration(Q1);
  q1->openNeighborhoodRadius = 0.5;
  AddConfigurationToCover(q1);
  RemoveConfigurationFromCover(q1);

  //#########################################################################
  //TEST2: Delete middle element
  //#########################################################################
  Configuration *q2 = new Configuration(Q1);
  Configuration *q3 = new Configuration(Q1);
  q2->openNeighborhoodRadius = 0.5;
  q3->openNeighborhoodRadius = 0.5;

  AddConfigurationToCover(q2);
  AddConfigurationToCover(q3);
  RemoveConfigurationFromCover(q2);
  RemoveConfigurationFromCover(q3);
  //#########################################################################

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
      Q1->printState(state_start);
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
    //#########################################################################
  }else{
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
  //#########################################################################
  //Parent data if available
  //#########################################################################
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

  //checkValidity();
  OMPL_INFORM("%s: ready with %lu states in datastructure (start radius: %f)", getName().c_str(), nearest_neighborhood->size(), q_start->GetRadius());

}
void QuotientCover::clear()
{
  BaseT::clear();
  totalVolumeOfCover = 0.0;

  //Nearestneighbors
  if(nearest_neighborhood) nearest_neighborhood->clear();
  if(nearest_configuration) nearest_configuration->clear();

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
  vertexToNormalizedIndexStdMap.clear();
  normalizedIndexToVertexStdMap.clear();
  index_ctr = 0;
  isConnected = false;
  saturated = false;
  firstRun = true;

  if(q_start){
    q_start->Clear();
  }
  if(q_goal){
    q_goal->Clear();
  }

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

  if(q->GetRadius() <= 0){
    std::cout << "[WARNING] Tried adding a zero-measure neighborhood." << std::endl;
    QuotientCover::Print(q, false);
    exit(0);
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
    OMPL_ERROR("Found no neighbors for Configuration");
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
      const double dqqk = metric->DistanceNeighborhoodNeighborhood(qk, q);
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

  uint Ne = GetNumberOfEdges(q);

  if(!q->isStart && Ne<=0){

    std::cout << "Detected no neighbors. This is usually a problem of the Interpolate() function, which has probably interpolated to a configuration which lies OUTSIDE of our cover (i.e. we went outside of the neighborhood we started with)." << std::endl;
    QuotientCover::Print(q, false);
    OMPL_ERROR("No Neighbors -- Cannot add to cover (Needs to be one single connected component)");
    std::cout << "NearestK found " << neighbors.size() << " neighbors." << std::endl;
    Configuration *qn = nearest_neighborhood->nearest(q);
    std::cout << "Distance to nearest NBH: " << metric->DistanceNeighborhoodNeighborhood(q, qn) << std::endl;
    std::cout << "Distance to nearest Config: " << metric->DistanceConfigurationConfiguration(q, qn) << std::endl;
    for(uint k = 0; k < neighbors.size(); k++){
      Configuration *qk = neighbors.at(k);
      std::cout << "NEIGHBOR " << k << "/" << neighbors.size()<< ":";
      if(IsNeighborhoodInsideNeighborhood(qk, q)){
        std::cout << "is inside current NBH (and has been flagged for removal)";
      }else{
        const double dqqk = metric->DistanceNeighborhoodNeighborhood(qk, q);
        const double dqk = metric->DistanceConfigurationConfiguration(qk, q);
        if(dqqk < 1e-10){
          std::cout << "OK (added edge)";
          AddEdge(q, qk);
        }else{
          std::cout << "Edge not added because distnace of NBHs is: " << dqqk << ", distance of configs is: " << dqk;
        }
      }
      std::cout << "|" << std::endl;

    }
    exit(0);
  }


  return v;
}

int QuotientCover::GetNumberOfEdges(Configuration *q)
{
  if(q->index < 0) return 0;
  Vertex v = get(normalizedIndexToVertex, q->index);
  return boost::out_degree(v, graph);
}

QuotientCover::Configuration* QuotientCover::GetOutwardPointingConfiguration(Configuration *q_center)
{
  Configuration *q_inward_to_outward = new Configuration(GetQ1(), q_center->GetInwardPointingConfiguration());

  double radius = q_center->GetRadius();
  double distance_center_inward = metric->DistanceQ1(q_inward_to_outward, q_center);
  double step = (radius+distance_center_inward)/distance_center_inward;

  //Make sure that this stops at the boundary 
  metric->InterpolateQ1(q_inward_to_outward, q_center, step, q_inward_to_outward);

  if(ComputeNeighborhood(q_inward_to_outward)){
    return q_inward_to_outward;
  }else{
    return nullptr;
  }
}

void QuotientCover::AddEdge(Configuration *q_from, Configuration *q_to)
{
  double d = metric->DistanceQ1(q_from, q_to);
  EdgeInternalState properties(d);
  Vertex v_from = get(normalizedIndexToVertex, q_from->index);
  Vertex v_to = get(normalizedIndexToVertex, q_to->index);
  boost::add_edge(v_from, v_to, properties, graph);

  q_from->UpdateRiemannianCenterOfMass(this, q_to);
  q_to->UpdateRiemannianCenterOfMass(this, q_from);

  q_from->number_of_neighbors++;
  q_to->number_of_neighbors++;
}

bool QuotientCover::EdgeExists(Configuration *q_from, Configuration *q_to)
{
  Vertex v_from = get(normalizedIndexToVertex, q_from->index);
  Vertex v_to = get(normalizedIndexToVertex, q_to->index);
  return boost::edge(v_from, v_to, graph).second;
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
  put(vertexToNormalizedIndex, v, index_ctr);
  put(normalizedIndexToVertex, index_ctr, v);
  graph[v]->index = index_ctr;
  //std::cout << "AddConfiguration idx ctr " << graph[v]->index << std::endl;
  index_ctr++;

  //###########################################################################
  //(2) add to nearest neighbor structure
  //###########################################################################
  nearest_neighborhood->add(q);
  nearest_configuration->add(q);

  //###########################################################################
  //(3) add to PDF
  //###########################################################################
  AddConfigurationToPDF(q);
  q->goal_distance = metric->DistanceConfigurationConfiguration(q, q_goal);

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
  const Vertex vq = get(normalizedIndexToVertex, q->index);

  //###########################################################################
  //(1) Remove from PDF
  //###########################################################################
  pdf_all_configurations.remove(static_cast<PDF_Element*>(q->GetPDFElement()));
  if(!q->isSufficientFeasible){
    pdf_necessary_configurations.remove(static_cast<PDF_Element*>(q->GetNecessaryPDFElement()));
  }

  //###########################################################################
  //(2) Remove from nearest neighbors structure
  //###########################################################################
  nearest_neighborhood->remove(q);
  nearest_configuration->remove(q);

  //###########################################################################
  //(3) Remove from cover graph
  //###########################################################################
  //check that they do not exist in normalizedIndexToVertex anymore
  vertexToNormalizedIndexStdMap.erase(vq);
  normalizedIndexToVertexStdMap.erase(q->index);

  index_ctr--; //points to last element
  if(q->index == index_ctr){
    //delete last element
  }else{
    Vertex v_last_in_graph = normalizedIndexToVertexStdMap[index_ctr];
    Configuration *q_last = graph[v_last_in_graph];

    normalizedIndexToVertexStdMap.erase(q_last->index);

    std::cout << "[REMOVE] idx " << q_last->index << "->" << q->index << std::endl;
    q_last->index = q->index;
    //vertexToNormalizedIndexStdMap[v_last_in_graph] = q->index;
    // normalizedIndexToVertexStdMap[q_last->index] = v_last_in_graph;
    put(vertexToNormalizedIndex, v_last_in_graph, q_last->index);
    put(normalizedIndexToVertex, q_last->index, v_last_in_graph);
  }

  //the last vertex index needs to point to the just purged index to obtain a
  //normalized index

  // put(vertexToNormalizedIndex, v, index_ctr);
  // put(normalizedIndexToVertex, index_ctr, v);
  // graph[v]->index = index_ctr;
  // index_ctr++;

  boost::clear_vertex(vq, graph);
  boost::remove_vertex(vq, graph);
  //erase entry from indexmap
    
  //###########################################################################
  //(4) delete q
  //###########################################################################
  q->Remove(Q1);
  delete q;
  q=nullptr;
}

//All edges pointing to q_from should be rerouted to point to q_to
void QuotientCover::RerouteEdgesFromTo(Configuration *q_from, Configuration *q_to)
{
  Vertex v_from = get(normalizedIndexToVertex, q_from->index);
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

    double d = DistanceInnerRobotToObstacle(q->state);
    q->SetRadius(d);
    if(q->GetRadius()<=minimum_neighborhood_radius){
      if(verbose) std::cout << "[ComputeNeighborhood] State Rejected: Radius Too Small (radius="<< q->GetRadius() 
        << ", minimum=" << minimum_neighborhood_radius << ")" << std::endl;
      q->SetRadius(minimum_neighborhood_radius);
      if(verbose) std::cout << "[ComputeNeighborhood] Set Radius to Minimal Radius (radius="<< q->GetRadius() << std::endl;
      // q->Remove(Q1);
      // q=nullptr;
      // return false;
    }
  }else{
    if(verbose) std::cout << "[ComputeNeighborhood] State Rejected: Infeasible" << std::endl;
    q->Remove(Q1);
    q=nullptr;
    return false;
  }

  return true;
}

bool QuotientCover::IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn)
{
  return (metric->DistanceConfigurationNeighborhood(q, qn) <= 1e-10);
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

//#############################################################################
//#############################################################################
//Sampling Configurations (on quotient space)
//#############################################################################
//#############################################################################

void QuotientCover::SampleGoal(Configuration *q)
{
  q->state = Q1->cloneState(q_goal->state);
}

void QuotientCover::SampleUniform(Configuration *q)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q->state);
  }else{
    ob::State *stateX1 = X1->allocState();
    ob::State *stateQ0 = Q0->allocState();
    X1_sampler->sampleUniform(stateX1);
    static_cast<og::QuotientCover*>(parent)->SampleUniformQuotientCover(stateQ0);
    MergeStates(stateQ0, stateX1, q->state);
    X1->freeState(stateX1);
    Q0->freeState(stateQ0);
  }
}

QuotientCover::Configuration* QuotientCover::SampleNeighborhoodBoundary(Configuration *q_center)
{
  const double radius = q_center->GetRadius();
  if(radius < minimum_neighborhood_radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "Configuration " << std::endl;
    Print(q_center);
    std::cout << "radius: " << radius << std::endl;
    std::cout << "min distance: " << minimum_neighborhood_radius << std::endl;
    std::cout << std::string(80, '#') << std::endl;
    exit(0);
  }

  Configuration *q_next = new Configuration(GetQ1());

  GetQ1SamplerPtr()->sampleUniformNear(q_next->state, q_center->state, q_center->GetRadius());
  double d = metric->DistanceConfigurationConfiguration(q_next, q_center);

  while(d < 1e-10){
    GetQ1SamplerPtr()->sampleUniformNear(q_next->state, q_center->state, q_center->GetRadius());
    d = metric->DistanceConfigurationConfiguration(q_next, q_center);
  }

  metric->InterpolateQ1(q_center, q_next, radius/d, q_next);

  return q_next;
}

void QuotientCover::SampleUniformQuotientCover(ob::State *state_random) 
{
  Configuration *q_coset = pdf_all_configurations.sample(rng_.uniform01());

  Q1_sampler->sampleUniformNear(state_random, q_coset->state, q_coset->GetRadius());
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
  double d = Q1->distance(q_coset->state, state_random);
  Q1->getStateSpace()->interpolate(q_coset->state, state_random, q_coset->GetRadius()/d, state_random);
}
QuotientCover::Configuration* QuotientCover::SampleNeighborhoodBoundaryUniformNear(Configuration *q_center, const Configuration* q_near, const double radius)
{
  Configuration *q_next = new Configuration(Q1);

  //Sample in Ambient Space
  GetQ1SamplerPtr()->sampleUniformNear(q_next->state, q_near->state /*mean*/, radius);

  ProjectConfigurationOntoNeighborhoodBoundary(q_center, q_next);

  if(ComputeNeighborhood(q_next)){
    return q_next;
  }else{
    return nullptr;
  }
}

//#############################################################################
// Cover Set Methods
//#############################################################################

bool QuotientCover::IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs)
{
  double distance_centers = metric->DistanceConfigurationConfiguration(lhs, rhs);
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

void QuotientCover::RewireConfiguration(Configuration *q)
{
  std::vector<Configuration*> neighbors;
  Vertex v = get(normalizedIndexToVertex, q->index);
  uint K = boost::degree(v, graph)+1;

  nearest_neighborhood->nearestK(q, K, neighbors);
  Configuration *qn = neighbors.at(K-1);
  double dn = metric->DistanceNeighborhoodNeighborhood(q, qn);
  if(dn <= 1e-10){
    AddEdge(q, qn);
  }
}

// void QuotientCover::CheckConfigurationIsOnBoundary(Configuration *q_boundary, Configuration *q)
// {
//   double d_outcome = metric->DistanceConfigurationConfiguration(q_boundary, q);
//   if(fabs(d_outcome - q->GetRadius())>1e-10)
//   {
//     std::cout << "Point not on boundary!" << std::endl;
//     QuotientCover::Print(q, false);
//     QuotientCover::Print(q_boundary, false);
//     std::cout << "radius neighborhood: " << q->GetRadius() << std::endl;
//     std::cout << "dist to point: " << d_outcome << std::endl;
//     exit(1);
//   }
// }

QuotientCover::Configuration* QuotientCover::NearestNeighborhood(const Configuration *q) const
{
  return nearest_neighborhood->nearest(const_cast<Configuration*>(q));
}
QuotientCover::Configuration* QuotientCover::NearestConfiguration(const Configuration *q) const
{
  return nearest_configuration->nearest(const_cast<Configuration*>(q));
}

void QuotientCover::ProjectConfigurationOntoNeighborhoodBoundary(const Configuration *q_center, Configuration* q_projected)
{
  const double d_center_to_proj = metric->DistanceConfigurationConfiguration(q_center, q_projected);
  double step_size = q_center->GetRadius()/d_center_to_proj;
  metric->Interpolate(q_center, q_projected, step_size, q_projected);
}

QuotientCover::Configuration* QuotientCover::NearestConfigurationOnBoundary(Configuration *q_center, const Configuration* q_outside)
{
  Configuration *q_projected = new Configuration(Q1);
  metric->Interpolate(q_center, q_outside, q_projected);

  if(ComputeNeighborhood(q_projected)){
    return q_projected;
  }else{
    return nullptr;
  }
}


bool QuotientCover::GetSolution(ob::PathPtr &solution)
{
  if(!isConnected){
    Configuration* qn = NearestNeighborhood(q_goal);
    double d_goal = metric->DistanceNeighborhoodNeighborhood(qn, q_goal);
    if(d_goal < 1e-10){
      v_goal = AddConfigurationToCover(q_goal);
      AddEdge(q_goal, qn);
      isConnected = true;
    }
  }
  if(isConnected){
    auto gpath(std::make_shared<PathGeometric>(Q1));
    //shortest_path_start_goal.clear();
    //shortest_path_start_goal_necessary_vertices.clear();

    //############################################################################
    //DEBUG
    // std::cout << std::string(80, '-') << std::endl;
    // std::cout << "found goal" << std::endl;
    // std::cout << "distance: " << d_goal << std::endl;
    // QuotientCover::Print(q_start, false);
    // QuotientCover::Print(q_goal, false);
    // std::cout << std::string(80, '-') << std::endl;
    //############################################################################

    if(v_start != get(normalizedIndexToVertex, q_start->index)){
      std::cout << "start index wrong:" << std::endl;
      std::cout << v_start << " vs " << q_start->index << std::endl;
      exit(0);
    }
    if(v_goal != get(normalizedIndexToVertex, q_goal->index)){
      std::cout << "goal index wrong:" << std::endl;
      std::cout << v_goal << " vs " << q_goal->index << std::endl;
      exit(0);
    }
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
  return nearest_configuration;
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
  const Vertex v_source = get(normalizedIndexToVertex, q_source->index);
  const Vertex v_sink = get(normalizedIndexToVertex, q_sink->index);
  std::vector<const Configuration*> q_path;
  if(v_source == v_sink){
    q_path.push_back(q_source);
    return q_path;
  }
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
  auto predecessor = boost::make_iterator_property_map(prev.begin(), vertexToNormalizedIndex);

  //check that vertices exists in graph
  if(verbose>3) std::cout << std::string(80, '-') << std::endl;
  if(verbose>3) std::cout << "searching from " << get(vertexToNormalizedIndex, v_source) << " to " << get(vertexToNormalizedIndex, v_sink) << std::endl;
  if(verbose>3) std::cout << "Max vertices: " << boost::num_vertices(graph) << ", idx ctr=" << index_ctr << std::endl;
  if(verbose>3) Q1->printState(graph[v_source]->state);
  if(verbose>3) Q1->printState(graph[v_sink]->state);
  //if(verbose>3) std::cout << graph << std::endl;

  try{
    boost::astar_search(graph, v_source,
                    [this, v_sink](const Vertex &v)
                    {
                        return ob::Cost(metric->DistanceConfigurationConfiguration(graph[v], graph[v_sink]));
                    },
                      predecessor_map(predecessor)
                      .weight_map(weight)
                      .visitor(astar_goal_visitor<Vertex>(v_sink))
                      .vertex_index_map(vertexToNormalizedIndex)
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
    for(Vertex v = v_sink;; v = prev[get(vertexToNormalizedIndex, v)])
    {
      path.push_back(v);
      // if(verbose>3)std::cout << std::string(80, '-') << std::endl;
      // if(verbose>3)std::cout << "v:" << graph[v]->index << std::endl;
      // if(verbose>3)std::cout << "idx:" << get(vertexToNormalizedIndex, v) << std::endl;
      if(graph[prev[get(vertexToNormalizedIndex, v)]]->index == graph[v]->index)
        break;
    }
    std::reverse(path.begin(), path.end());
  }catch(const std::exception& e){
    std::cerr << "ERROR: " << e.what() << std::endl;
    exit(-1);
  }catch(...){
    std::cerr << "ERROR: " << std::endl;
    exit(-1);
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

  if(boost::num_vertices(graph)<=0) return;

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
  std::cout << " | neighbors : " << (q->number_of_neighbors) << std::endl;
  std::cout << (q->isGoal?" | GOAL STATE":"") << (q->isStart?" | START STATE":"") << std::endl;
  if(!stopOnError) return;
  if(q->index < 0)
  {
    std::cout << "[### STATE NOT MEMBER OF COVER]" << std::endl;
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
