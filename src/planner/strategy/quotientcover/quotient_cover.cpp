#include "common.h"
#include "quotient_cover.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

QuotientChartCover::QuotientChartCover(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QuotientChartCover"+std::to_string(id));
  if (!nearest_cover){
    nearest_cover.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_cover->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceNeighborhoodNeighborhood(a,b);
                              });
  }
  if (!nearest_vertex){
    nearest_vertex.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_vertex->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return DistanceConfigurationConfiguration(a,b);
                              });
  }

  const double maxBias = 0.9;
  if(goalBias+voronoiBias >= maxBias)
  {
    std::cout << "goal and voronoi bias are too big." << std::endl;
    std::cout << "goal bias: " << goalBias << std::endl;
    std::cout << "voronoi bias: " << voronoiBias << std::endl;
    std::cout << "(should be below "<< maxBias << ")" << std::endl;
    exit(0);
  }
}

QuotientChartCover::~QuotientChartCover(void)
{
}
//#############################################################################
//SETUP
//#############################################################################

void QuotientChartCover::setup(void)
{
  if (pdef_){
    //#########################################################################
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      OMPL_ERROR("%s: Did not specify optimization function.", getName().c_str());
      exit(0);
    }
    //#########################################################################
    //Parent data if available
    //#########################################################################
    Configuration *coset_start = nullptr;
    Configuration *coset_goal = nullptr;
    if(parent != nullptr)
    {
      coset_start = static_cast<og::QuotientChartCover*>(parent)->q_start;
      coset_goal = static_cast<og::QuotientChartCover*>(parent)->q_goal;
    }
    //#########################################################################
    //Adding start configuration
    //#########################################################################
    if(const ob::State *state_start = pis_.nextStart()){
      q_start = CreateConfigurationFromStateAndCoset(state_start, coset_start);
      if(q_start == nullptr){
        OMPL_ERROR("%s: Could not add start state!", getName().c_str());
        exit(0);
      }
      v_start = AddConfigurationToCover(q_start);
    }else{
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      exit(0);
    }

    //#########################################################################
    //Adding goal configuration
    //#########################################################################
    if(const ob::State *state_goal = pis_.nextGoal()){
      q_goal = CreateConfigurationFromStateAndCoset(state_goal, coset_goal);
      if(q_goal == nullptr){
        OMPL_ERROR("%s: Could not add goal state!", getName().c_str());
        exit(0);
      }
    }else{
      OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
      exit(0);
    }

    q_start->isStart = true;
    q_goal->isGoal = true;

    //#########################################################################
    //Check saturation
    //#########################################################################
    if(q_start->GetRadius() == std::numeric_limits<double>::infinity())
    {
      OMPL_INFORM("Note: start state covers quotient-space.");
      saturated = true;
    }

    //#########################################################################
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nearest_cover->size());
    setup_ = true;
    std::cout << "start: " << get(vertexToIndex, v_start) << " goal: " << get(vertexToIndex, v_goal) << std::endl;
  }else{
    setup_ = false;
  }

}

//#############################################################################
//Configuration methods
//#############################################################################
QuotientChartCover::Vertex QuotientChartCover::AddConfigurationToCover(Configuration *q)
{
  //get all neighbors before adding q (otherwise q might be neighbor of itself)
  std::vector<Configuration*> neighbors = GetConfigurationsInsideNeighborhood(q);

  //(1) add to cover graph
  Vertex v = boost::add_vertex(q, graph);
  graph[v]->number_attempted_expansions = 0;
  graph[v]->number_successful_expansions = 0;

  //assign vertex to a unique index
  put(vertexToIndex, v, index_ctr);
  put(indexToVertex, index_ctr, v);
  graph[v]->index = index_ctr;
  index_ctr++;

  //(2) add to nearest neighbor structure
  nearest_cover->add(q);
  nearest_vertex->add(q);

  //(3) add to PDF
  PDF_Element *q_element = pdf_all_configurations.add(q, q->GetImportance());
  q->SetPDFElement(q_element);
  if(!q->isSufficientFeasible){
    PDF_Element *q_necessary_element = pdf_necessary_configurations.add(q, q->GetRadius());
    q->SetNecessaryPDFElement(q_necessary_element);
  }

  if(verbose>0) std::cout << std::string(80, '-') << std::endl;
  if(verbose>0) std::cout << "*** ADD VERTEX " << q->index << std::endl;

  if(q->parent_neighbor != nullptr){
    AddEdge(q, q->parent_neighbor);
  }

  //Clean UP and Connect
  //(1) Remove All Configurations with neighborhoods inside current neighborhood
  //(2) Add Edges to All Configurations with centers inside the current neighborhood 

  if(verbose>0) std::cout << "Vertex: " << q->index << " has number of neighbors: " << neighbors.size() << std::endl;
  if(verbose>0) std::cout << std::string(80, '-') << std::endl;
  for(uint k = 0; k < neighbors.size(); k++){
    Configuration *qn = neighbors.at(k);
    if(qn==q){
      std::cout << "configuration equals neighbor." << std::endl;
      exit(0);
    }
    //do not connect to parent, we already connected that
    if(qn == q->parent_neighbor) continue;
    if(IsNeighborhoodInsideNeighborhood(qn, q))
    {
      //do not delete start/goal
      if(qn->isStart){
        AddEdge(q, qn);
      }else if(qn->isGoal){
        AddEdge(q, qn);
      }else{
        //old constellation
        //v1 -------                                    |
        //          \                                   |
        //v2-------- vn ------- vq                      |
        //          /                                   |
        //v3--------                                    |
        //
        //after edge adding 
        //v1 -------------------                        |
        //          \           \                       |
        //v2-------- vn ------- vq                      |
        //          /           /                       |
        //v3--------------------                        |
        //after edge removal 
        //v1 -------------------                        |
        //                      \                       |
        //v2------   vn   ----- vq                      |
        //        \------/      /                       |
        //v3--------------------                        |
        //after vertex removal
        //v1 ---------------                            |
        //                  \                           |
        //v2--------------- vq                          |
        //                  /                           |
        //v3----------------                            |
        //
        //get all edges into qn, and rewire them to q
        Vertex vn = get(indexToVertex, qn->index);
        OEIterator edge_iter, edge_iter_end, next; 
        boost::tie(edge_iter, edge_iter_end) = boost::out_edges(vn, graph);

        vertex_index_type vi = get(vertexToIndex, v);
        vertex_index_type vni = get(vertexToIndex, vn);
        if(verbose>1) std::cout << "vertex " << qn->index << " needs to be removed" << std::endl;
        if(verbose>1) std::cout << "vertex " << vi << "(" << q->index << ") and neighbor " << vni << "(" << qn->index << ")" << std::endl;
        if(verbose>1) std::cout << "removing " << boost::out_degree(vn, graph) << " edges from vertex " << vni << std::endl;

        for(next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
        {
          //add v_target to v
          ++next;
          const Vertex v_target = boost::target(*edge_iter, graph);
          //segfault, because v_target points to an already removed vertex!
          if(verbose>1) std::cout << "REWIRE EDGE from " << vni << "(" << get(vertexToIndex, vn) << ") to " << get(vertexToIndex, v_target) << "(" << graph[v_target]->index << ")" << std::endl;
          double d = DistanceQ1(graph[v_target], q);
          EdgeInternalState properties(d);

          if(v_target!=v)
          {
            //do not add to itself
            if(verbose>1) std::cout << "add " << get(vertexToIndex, v) << "-" << get(vertexToIndex, v_target) 
              << "(" << graph[v_target]->index << ")" << std::endl;

            //vertex might be invalidated
            boost::add_edge(v_target, v, properties, graph);
          }
        }
        boost::clear_vertex(vn, graph);
        RemoveConfigurationFromCover(qn);
      }
    }else{
      AddEdge(q, qn);
    }
  }
  return v;
}
void QuotientChartCover::AddEdge(Configuration *q_from, Configuration *q_to)
{
  //std::cout << "adding edge " << q_from->index << " - " << q_to->index << std::endl;
  double d = DistanceQ1(q_from, q_to);
  EdgeInternalState properties(d);
  Vertex v_from = get(indexToVertex, q_from->index);
  Vertex v_to = get(indexToVertex, q_to->index);
  boost::add_edge(v_from, v_to, properties, graph);
}

void QuotientChartCover::RemoveConfigurationFromCover(Configuration *q)
{
  //(1) Remove from PDF
  pdf_all_configurations.remove(static_cast<PDF_Element*>(q->GetPDFElement()));
  if(!q->isSufficientFeasible){
    pdf_necessary_configurations.remove(static_cast<PDF_Element*>(q->GetNecessaryPDFElement()));
  }
  //(2) Remove from nearest neighbors structure
  nearest_cover->remove(q);
  nearest_vertex->remove(q);

  //(3) Remove from cover graph
  q->Remove(Q1);

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "*** REMOVE VERTEX " << q->index << std::endl;

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

  boost::remove_vertex(vq, graph);

  delete q;
  q=nullptr;
}


QuotientChartCover::Configuration* QuotientChartCover::CreateConfigurationFromStateAndCoset(const ob::State *state, Configuration *q_coset)
{
  Configuration *q = new Configuration(Q1, state);

  //cannot create configurations inside cover
  if(IsConfigurationInsideCover(q)){
    q->Remove(Q1);
    return nullptr;
  }
  q->coset = q_coset;

  bool feasible = Q1->isValid(q->state);

  if(feasible){
    if(IsOuterRobotFeasible(q->state))
    {
      q->isSufficientFeasible = true;
      q->SetRadius(DistanceOuterRobotToObstacle(q->state));
    }else{
      q->SetRadius(DistanceInnerRobotToObstacle(q->state));
    }
    if(q->GetRadius()<threshold_clearance){
      q->Remove(Q1);
      return nullptr;
    }
  }else{
    q->Remove(Q1);
    return nullptr;
  }

  return q;
}
void QuotientChartCover::RemoveConfigurationsFromCoverCoveredBy(Configuration *q)
{
  //If the neighborhood of q is a superset of any other neighborhood, then delete
  //the other neighborhood, and rewire the graph
  std::vector<Configuration*> neighbors = GetConfigurationsInsideNeighborhood(q);

  for(uint k = 0; k < neighbors.size(); k++){

    Configuration *qn = neighbors.at(k);

    //do not delete start/goal
    if(qn->isStart) continue;
    if(qn->isGoal) continue;

    if(IsNeighborhoodInsideNeighborhood(qn, q))
    {
      RemoveConfigurationFromCover(qn);
    }
  }
}


bool QuotientChartCover::IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn)
{
  return (DistanceConfigurationConfiguration(q, qn) <= 0);
}

bool QuotientChartCover::IsConfigurationInsideCover(Configuration *q)
{
  std::vector<Configuration*> neighbors;

  nearest_cover->nearestK(q, 2, neighbors);

  if(neighbors.size()<=1) return false;

  return IsConfigurationInsideNeighborhood(q, neighbors.at(0)) && IsConfigurationInsideNeighborhood(q, neighbors.at(1));
}

void QuotientChartCover::Grow(double t)
{
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );
  //#########################################################################
  //Do not grow the cover if it is saturated, i.e. it cannot be expanded anymore
  // --- we have successfully computed Q1_{free}, the free space of Q1
  //#########################################################################
  if(saturated) return;

  //#########################################################################
  //Sample a configuration different from the current cover
  //#########################################################################
  Configuration *q_random = SampleValid(ptc);
  if(q_random == nullptr) return;

  AddConfigurationToCover(q_random);

}

QuotientChartCover::Configuration* QuotientChartCover::EstimateBestNextState(Configuration *q_last, Configuration *q_current)
{
  std::cout << "NYI" << std::endl;
  exit(0);
    // uint K_samples = 3; //how many samples to test for best direction (depends maybe also on radius)

    // Configuration *q_next = new Configuration(Q1);

    // if(verbose>1){
    //   std::cout << "from" << std::endl;
    //   Q1->printState(q_last->state);
    //   std::cout << "to" << std::endl;
    //   Q1->printState(q_current->state);
    // }

    // double d_last_to_current = Distance(q_last, q_current);
    // double radius_current = q_current->GetRadius();
    // Q1->getStateSpace()->interpolate(q_last->state, q_current->state, 1 + radius_current/d_last_to_current, q_next->state);

    // //#######################################################################
    // // We sample the distance function to obtain K samples {q_1,\cdots,q_K}.
    // // For each $q_k$ we compute the value of the distance function dk_radius.
    // // Then we employ two strategies to choose the next sample to follow:
    // //
    // //either follow isolines : min( fabs(radius_k_sample - current_radius) )
    // //or follow the steepest ascent: max(radius_k_sample)

    // double radius_best = DistanceInnerRobotToObstacle(q_next->state);
    // if(verbose>1){
    //   std::cout << "next" << std::endl;
    //   Q1->printState(q_next->state);
    //   std::cout << "radius " << radius_best << std::endl;
    // }

    // //double radius_best = q_next->GetRadius();
    // double SAMPLING_RADIUS = 0.1*radius_current;
    // double radius_ratio = radius_best / radius_current;
    // if(radius_ratio > 1){
    //   //we encountered a bigger radius neighborhood. this is great, we should go
    //   //into that direction
    //   if(!AddConfiguration(q_next))
    //   {
    //     return nullptr;
    //   }
    //   return q_next;
    // }else{
    //   //if next neighborhood is exceptionally small, try to sample more broadly.
    //   //Otherwise sample smaller. This corresponds to having more speed in the
    //   //momentum of sampling.
    //   if(radius_ratio > 0.1){
    //     SAMPLING_RADIUS = (1.0-radius_ratio+0.1)*radius_current;
    //   }else{
    //     SAMPLING_RADIUS = radius_current;
    //   }
    // }

    // //#######################################################################
    // //keep q_next constant
    // q_next = const_cast<Configuration*>(q_next);
    // Configuration *q_best = q_next;

    // for(uint k = 0; k < K_samples; k++)
    // {
    //   //obtain sample q_k, and radius radius_k
    //   Configuration *q_k = new Configuration(Q1);
    //   Q1_sampler->sampleUniformNear(q_k->state, q_next->state, SAMPLING_RADIUS);
    //   double d_current_to_k = DistanceQ1(q_k, q_current);
    //   Q1->getStateSpace()->interpolate(q_current->state, q_k->state, radius_current/d_current_to_k, q_k->state);

    //   double radius_next = DistanceInnerRobotToObstacle(q_k->state);

    //   if(verbose>1){
    //     std::cout << "q_k" << std::endl;
    //     Q1->printState(q_k->state);
    //     std::cout << "radius: " << radius_next << std::endl;
    //   }

    //   if(q_best->isSufficientFeasible && !q_k->isSufficientFeasible)
    //   {
    //     q_best = q_k;
    //     radius_best = radius_next;
    //   }else{
    //     if(radius_next > radius_best)
    //     {
    //       q_best = q_k;
    //       radius_best = radius_next;
    //     }else{
    //       q_k->Remove(Q1);
    //       continue;
    //     }
    //   }
    // }
    // if(!AddConfiguration(q_best))
    // {
    //   return nullptr;
    // }
    // //#########################################################################
    // //return q_next
    // //#########################################################################
    // return q_best;
}
//#############################################################################
//Sampling Configurations (on quotient space)
//#############################################################################

void QuotientChartCover::SampleGoal(Configuration *q)
{
  q->state = Q1->cloneState(q_goal->state);
  if(parent == nullptr){
  }else{
    q->coset = static_cast<og::QuotientChartCover*>(parent)->q_goal;
  }
}

void QuotientChartCover::SampleUniform(Configuration *q)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q->state);
  }else{
    ob::State *stateX1 = X1->allocState();
    ob::State *stateQ0 = Q0->allocState();
    X1_sampler->sampleUniform(stateX1);
    q->coset = static_cast<og::QuotientChartCover*>(parent)->SampleQuotientCover(stateQ0);
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

//#############################################################################
//Sampling Configurations (On boundary of cover)
//#############################################################################

QuotientChartCover::Configuration* QuotientChartCover::SampleBoundary(std::string type)
{
  Configuration *q_random = new Configuration(Q1);
  if(type == "voronoi"){
    SampleUniform(q_random);
  }else if(type == "goal"){
    SampleGoal(q_random);
  }else if(type == "boundary"){
    Configuration *q_nearest = pdf_all_configurations.sample(rng_.uniform01());
    q_nearest->number_attempted_expansions++;
    pdf_all_configurations.update(static_cast<PDF_Element*>(q_nearest->GetPDFElement()), q_nearest->GetImportance());

    //sample its boundary
    //sampleHalfBallOnNeighborhoodBoundary(q_random->state, q_nearest);
    sampleUniformOnNeighborhoodBoundary(q_random->state, q_nearest);
    q_random->parent_neighbor = q_nearest;
    return q_random;
  }else{
    std::cout << "sampling type " << type << " not recognized." << std::endl;
    exit(0);
  }

  Configuration *q_nearest = Nearest(q_random);
  Connect(q_nearest, q_random, q_random);
  q_random->parent_neighbor = q_nearest;
  return q_random;
}

//#############################################################################
// Main Sample Function
//#############################################################################


QuotientChartCover::Configuration* QuotientChartCover::Sample(){
  Configuration *q_random;
  if(!hasSolution){
    double r = rng_.uniform01();
    if(r<goalBias){
      q_random = SampleBoundary("goal");
    }else{
      if(r < (goalBias + voronoiBias)){
        q_random = SampleBoundary("voronoi");
      }else{
        q_random = SampleBoundary("boundary");
      }
    }
  }else{
    std::cout << "hasSolution sampler NYI"<< std::endl;
    exit(0);
  }
  return q_random;
}
QuotientChartCover::Configuration* QuotientChartCover::SampleValid(ob::PlannerTerminationCondition &ptc)
{
  Configuration *q = nullptr;
  while(!ptc){
    q = Sample();
    //ignore samples inside cover (rejection sampling)
    if(q == nullptr) continue;
    if(IsConfigurationInsideCover(q)){
      q->Remove(Q1);
      q = nullptr;
      continue;
    }
    bool feasible = Q1->isValid(q->state);
    if(feasible){
      if(IsOuterRobotFeasible(q->state))
      {
        q->isSufficientFeasible = true;
        q->SetRadius(DistanceOuterRobotToObstacle(q->state));
      }else{
        q->SetRadius(DistanceInnerRobotToObstacle(q->state));
      }
      if(q->GetRadius()<threshold_clearance){
        q->Remove(Q1);
        q = nullptr;
        continue;
      }
    }else{
      q->Remove(Q1);
      q = nullptr;
      continue;
    }
    break;
  }
  return q;
}

QuotientChartCover::Configuration* QuotientChartCover::SampleQuotientCover(ob::State *state) const
{
  std::cout << "sample quotient cover" << std::endl;
  std::cout << "NYI" << std::endl;
  exit(0);
}

bool QuotientChartCover::sampleUniformOnNeighborhoodBoundary(ob::State *state, const Configuration *center)
{
  //sample on boundary of open neighborhood
  // (1) first sample gaussian point qk around q
  // (2) project qk onto neighborhood
  // (*) this works because gaussian is symmetric around origin
  // (*) this does not work when qk is near to q, so we need to sample as long
  // as it is not near
  //
  double radius = center->openNeighborhoodRadius;

  double dist_q_qk = 0;

  //@DEBUG
  if(epsilon_min_distance >= radius){
    OMPL_ERROR("neighborhood is too small to sample the boundary.");
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "states currently considered:" << std::endl;
    for(uint k = 0; k < pdf_all_configurations.size(); k++){
      Configuration *q = pdf_all_configurations[k];
      std::cout << "state " << k << " with radius " << q->GetRadius() << " has values:" << std::endl;
      Q1->printState(q->state);
    }
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "neighborhood radius: " << radius << std::endl;
    std::cout << "minimal distance to be able to sample: " << epsilon_min_distance << std::endl;
    std::cout << "state:" << std::endl;
    Q1->printState(center->state);
    exit(0);
  }

  //sample as long as we are inside the ball of radius epsilon_min_distance
  while(dist_q_qk <= epsilon_min_distance){
    Q1_sampler->sampleGaussian(state, center->state, 1);
    dist_q_qk = si_->distance(center->state, state);
  }
  si_->getStateSpace()->interpolate(center->state, state, radius/dist_q_qk, state);

  return true;
}

//@brief: move from q_from to q_to until the neighborhood of q_from is
//intersected. Return the intersection point in q_out
void QuotientChartCover::Connect(const Configuration *q_from, const Configuration *q_to, Configuration *q_out)
{
  double radius = q_from->openNeighborhoodRadius;

  if(parent == nullptr)
  {
    double dist_qfrom_qto = DistanceConfigurationConfiguration(q_from, q_to);
    Q1->getStateSpace()->interpolate(q_from->state, q_to->state, radius/dist_qfrom_qto, q_out->state);
  }else{
    std::cout << "Connect() NYI" << std::endl;
    exit(0);

  }

}

bool QuotientChartCover::IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs)
{
  double distance_centers = DistanceQ1(lhs, rhs);
  double radius_rhs = rhs->GetRadius();
  double radius_lhs = lhs->GetRadius();
  return (radius_rhs > (radius_lhs + distance_centers));
}

std::vector<QuotientChartCover::Configuration*> QuotientChartCover::GetConfigurationsInsideNeighborhood(Configuration *q)
{
  std::vector<Configuration*> neighbors;
  nearest_vertex->nearestR(q, q->GetRadius()+threshold_clearance*0.5, neighbors);
  return neighbors;
}


QuotientChartCover::Configuration* QuotientChartCover::Nearest(Configuration *q) const
{
  if(parent == nullptr){
    return nearest_cover->nearest(q);
  }else{
    std::cout << "Nearest NYI" << std::endl;
    exit(0);
  }
}

//@TODO: should be fixed to reflect the distance on the underlying
//quotient-space, i.e. similar to QMPConnect. For now we use DistanceQ1
bool QuotientChartCover::Interpolate(const Configuration *q_from, Configuration *q_to)
{
  double d = DistanceQ1(q_from, q_to);
  double radius = q_from->GetRadius();
  double step_size = radius/d;
  si_->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_to->state);
  return true;
}


//#############################################################################
//Distance Functions
//#############################################################################

double QuotientChartCover::DistanceQ1(const Configuration *q_from, const Configuration *q_to)
{
  return Q1->distance(q_from->state, q_to->state);
}

double QuotientChartCover::DistanceX1(const Configuration *q_from, const Configuration *q_to)
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

double QuotientChartCover::DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to)
{
  if(parent == nullptr){
    //the very first quotient-space => usual metric
    return DistanceQ1(q_from, q_to);
  }else{
    if(q_to->coset == nullptr || q_from->coset == nullptr)
    {
      //could not find a coset on the quotient-space  => usual metric
      std::cout << "[WARNING] could not find coset for a configuration" << std::endl;
      return DistanceQ1(q_from, q_to);
    }
    //metric defined by distance on cover of quotient-space plus distance on the
    //subspace Q1/Q0
    og::QuotientChartCover *QuotientChartCover_parent = dynamic_cast<og::QuotientChartCover*>(parent);
    return QuotientChartCover_parent->DistanceConfigurationConfigurationCover(q_from->coset, q_to->coset)+DistanceX1(q_from, q_to);
  }
}

double QuotientChartCover::DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_to = q_to->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_to);
  return std::max(d - d_to, 0.0);
}
double QuotientChartCover::DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to)
{
  double d_from = q_from->GetRadius();
  double d_to = q_to->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_to);

  if(d!=d){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "NaN detected." << std::endl;
    std::cout << "d_from " << d_from << std::endl;
    std::cout << "d_to " << d_to << std::endl;
    std::cout << "d " << d << std::endl;
    std::cout << "configuration 1: " << std::endl;
    Q1->printState(q_from->state);
    std::cout << "configuration 2: " << std::endl;
    Q1->printState(q_to->state);
    std::cout << std::string(80, '-') << std::endl;

    exit(1);
  }
  double d_open_neighborhood_distance = std::max(d - d_from - d_to, 0.0); 
  return d_open_neighborhood_distance;
}

double QuotientChartCover::DistanceConfigurationConfigurationCover(const Configuration *q_from, const Configuration *q_to)
{
  std::cout << "shortest path distance on cover" << std::endl;
  std::cout << "NYI" << std::endl;
  exit(0);
}


void QuotientChartCover::Init()
{
  checkValidity();
}


void QuotientChartCover::clear()
{
  Planner::clear();
  if(nearest_cover){
    nearest_cover->clear();
  }
  if(nearest_vertex){
    nearest_vertex->clear();
  }
  hasSolution = false;
  q_start = nullptr;
  q_goal = nullptr;

  pis_.restart();
}

bool QuotientChartCover::GetSolution(ob::PathPtr &solution)
{
  //create solution only if we already found a path
  if(hasSolution){
    std::vector<Vertex> vpath = GetCoverPath(v_start, v_goal);
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*solution);
    gpath.clear();
    for(uint k = 0; k < vpath.size(); k++){
      gpath.append(graph[vpath.at(k)]->state);
    }
  }
  return hasSolution;
}

void QuotientChartCover::CopyChartFromSibling( QuotientChart *sibling, uint k )
{
  std::cout << "copy chart " << std::endl;
  std::cout << "NYI" << std::endl;
  exit(0);
}

QuotientChartCover::Configuration* QuotientChartCover::GetStartConfiguration() const
{
  return q_start;
}
QuotientChartCover::Configuration* QuotientChartCover::GetGoalConfiguration() const
{
  return q_goal;
}

const QuotientChartCover::PDF& QuotientChartCover::GetPDFNecessaryVertices() const
{
  return pdf_necessary_configurations;
}

const QuotientChartCover::PDF& QuotientChartCover::GetPDFAllVertices() const
{
  return pdf_all_configurations;
}

double QuotientChartCover::GetGoalBias() const
{
  return goalBias;
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


std::vector<QuotientChartCover::Vertex> QuotientChartCover::GetCoverPath(const Vertex& start, const Vertex& goal)
{
  std::vector<Vertex> path;

  std::cout << "finding path between " << start << " - " << goal << std::endl;
  Q1->printState(graph[start]->state);
  Q1->printState(graph[goal]->state);

    //vector<Vertex> p(num_vertices(G), graph_traits<G>::null_vertex()); //the predecessor array
  std::vector<Vertex> prev(boost::num_vertices(graph), boost::graph_traits<Graph>::null_vertex());
  auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost), get(boost::edge_bundle, graph));
  auto predecessor = boost::make_iterator_property_map(prev.begin(), vertexToIndex);

  try{
  boost::astar_search(graph, start,
                    [this, goal](const Vertex &v)
                    {
                        return ob::Cost(DistanceQ1(graph[v], graph[goal]));
                    },
                    //boost::predecessor_map(&prev[0])
                      predecessor_map(predecessor)
                      .weight_map(weight)
                      .visitor(astar_goal_visitor<Vertex>(goal))
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
    //std::cout << prev[get(vertexToIndex, goal)] << std::endl;
    for(Vertex v = goal;; v = prev[get(vertexToIndex, v)])
    {
      path.push_back(v);
      if(prev[get(vertexToIndex, v)] == v)
        break;
    }
    std::reverse(path.begin(), path.end());
    //list<Vertex>::iterator spi = shortest_path.begin();
  }

  // if (prev[goal.index] == goal){
  //   return path;
  // }
  // for(Vertex pos = goal; prev[pos] != pos; pos = prev[pos]){
  //   path.push_back(pos);
  // }
  // path.push_back(start);
  return path;
}


PlannerDataVertexAnnotated QuotientChartCover::getAnnotatedVertex(ob::State* state, double radius, bool sufficient) const
{
  PlannerDataVertexAnnotated pvertex(state);
  pvertex.SetLevel(GetLevel());
  pvertex.SetPath(GetChartPath());
  pvertex.SetOpenNeighborhoodDistance(radius);

  if(!state){
    std::cout << "vertex state does not exists" << std::endl;
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
PlannerDataVertexAnnotated QuotientChartCover::getAnnotatedVertex(Vertex vertex) const
{
  ob::State *state = (isLocalChart?si_->cloneState(graph[vertex]->state):graph[vertex]->state);
  return getAnnotatedVertex(state, graph[vertex]->GetRadius(), graph[vertex]->isSufficientFeasible);
}


void QuotientChartCover::getPlannerDataAnnotated(base::PlannerData &data) const
{
  std::cout << "graph has " << boost::num_vertices(graph) << " vertices (index ctr: " << index_ctr << ") and " << boost::num_edges(graph) << " edges." << std::endl;
  std::map<const Vertex, const ob::State*> vertexToStates;

  PlannerDataVertexAnnotated pstart = getAnnotatedVertex(v_start);
  vertexToStates[v_start] = pstart.getState();
  data.addStartVertex(pstart);

  foreach( const Vertex v, boost::vertices(graph))
  {
    if(vertexToStates.find(v) == vertexToStates.end()) {
      PlannerDataVertexAnnotated p = getAnnotatedVertex(v);
      data.addVertex(p);
      vertexToStates[v] = p.getState();
    }
    //otherwise vertex is a goal or start vertex and has already been added
  }
  foreach (const Edge e, boost::edges(graph))
  {
    const Vertex v1 = boost::source(e, graph);
    const Vertex v2 = boost::target(e, graph);

    const ob::State *s1 = vertexToStates[v1];
    const ob::State *s2 = vertexToStates[v2];
    PlannerDataVertexAnnotated p1(s1);
    PlannerDataVertexAnnotated p2(s2);
    data.addEdge(p1,p2);
  }
  std::cout << "added " << data.numVertices() << " vertices and " << data.numEdges() << " edges."<< std::endl;
}

