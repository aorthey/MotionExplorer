#include "common.h"
#include "gui/common.h"
#include "qcp.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
#include "planner/strategy/quotient/cover_expansion_strategy/expansion_goal.h"
#include "planner/strategy/quotient/cover_expansion_strategy/expansion_cache_goal.h"
#include "planner/strategy/quotient/cover_expansion_strategy/expansion_outwards.h"
#include "planner/strategy/quotient/cover_expansion_strategy/expansion_random_voronoi.h"
#include "planner/strategy/quotient/cover_expansion_strategy/expansion_random_boundary.h"

using namespace ompl::geometric;

//############################################################################
// Setup
//############################################################################
QCP::QCP(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QCP"+std::to_string(id));
  progressMadeTowardsGoal = true;
  SetMetric("shortestpath_simplified");
  //SetMetric("euclidean");
  expansion_strategy_goal = std::make_shared<CoverExpansionStrategyCacheGoal>(this);
  //expansion_strategy_goal = std::make_shared<CoverExpansionStrategyGoal>(this);
  expansion_strategy_outwards = std::make_shared<CoverExpansionStrategyOutwards>(this);
  expansion_strategy_random_voronoi = std::make_shared<CoverExpansionStrategyRandomVoronoi>(this);
  expansion_strategy_random_boundary = std::make_shared<CoverExpansionStrategyRandomBoundary>(this);
}

void QCP::setup()
{
  BaseT::setup();
}

void QCP::clear()
{
  progressMadeTowardsGoal = true;
  expansion_strategy_goal->Clear();
  expansion_strategy_outwards->Clear();
  expansion_strategy_random_boundary->Clear();
  expansion_strategy_random_voronoi->Clear();
  BaseT::clear();
}


//############################################################################
// Grow Functions
//############################################################################
void QCP::Grow(double t)
{
  if(firstRun){
    firstRun = false;
    Init();
    AddConfigurationToPriorityQueue(GetStartConfiguration());
  }
  if(saturated) return;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );

  if(!hasSolution){
    GrowWithoutSolution(ptc);
  }else{
    GrowWithSolution(ptc);
  }
}

void QCP::GrowWithoutSolution(ob::PlannerTerminationCondition &ptc)
{
  if(NearestToGoalHasChanged() || progressMadeTowardsGoal)
  {
    //std::cout << std::string(80, '%') << std::endl;
    progressMadeTowardsGoal = (expansion_strategy_goal->Step() > 0);
    if(!progressMadeTowardsGoal){
      expansion_strategy_goal->Clear();
    }
  }else{
    if(PriorityQueueCandidate_IsEmpty()){
      expansion_strategy_random_voronoi->Step();
    }else{
      expansion_strategy_outwards->Step();
    }
  }
}

void QCP::GrowWithSolution(ob::PlannerTerminationCondition &ptc)
{
  double r = rng_.uniform01();
  if(r <= rewireBias){
    RewireCover(ptc);
  }else{
    expansion_strategy_random_boundary->Step();
  }
}

//############################################################################
// Misc Functions
//############################################################################

void QCP::RewireCover(ob::PlannerTerminationCondition &ptc)
{
  //Configuration *q = pdf_connectivity_configurations.sample(rng_.uniform01());
  Configuration *q = GetConfigurationLowConnectivity();

  //find all vertices which intersect NBH, then check if they have an edge in
  //common. Then add one if they don't.
  //RewireConfiguration(q);
  //for(uint k = 0; k < q_neighbors.size(); k++){
  //  Configuration *qk = q_neighbors.at(k);
  //  if(!EdgeExists(q, qk)){
  //    pdf_connectivity_configurations.update(static_cast<PDF_Element*>(qk->GetConnectivityPDFElement()), ValueConnectivity(qk));
  //    AddEdge(q, qk);
  //  }
  //}
  if(q==nullptr){
    std::cout << "Configuration does not exist." << std::endl;
    QuotientCover::Print(q, false);
    exit(0);
  }

  //add one more edges
  std::vector<Configuration*> neighbors;
  Vertex v = get(normalizedIndexToVertex, q->index);
  uint K = boost::degree(v, graph)+1;
  nearest_neighborhood->nearestK(const_cast<Configuration*>(q), K, neighbors);

  if(neighbors.size()>=K){
    Configuration *qn = neighbors.at(K-1);
    double dn = GetMetric()->DistanceNeighborhoodNeighborhood(q, qn);
    if(dn <= 1e-10){
      AddEdge(q, qn);
    }
  }
}

double QCP::GetImportance() const
{
  //approximation of total volume covered
  //double percentageCovered = totalVolumeOfCover/Q1->getStateSpace()->getMeasure();
  //the more sampled, the less important this space becomes
  //return 1.0/(totalVolumeOfCover+1);

  if(!hasSolution && progressMadeTowardsGoal){
    //If we move towards goal, greedily allocate all computational resources
    return +dInf;
  }else{
    return 1.0/(boost::num_vertices(graph)+1);
  }
  //double importance = pow((double)2.0, level);
  //return importance;
}
