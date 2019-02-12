#include "common.h"
#include "gui/common.h"
#include "qcp.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
#include "planner/strategy/quotient/step_strategy/step.h"

using namespace ompl::geometric;

//############################################################################
// Setup
//############################################################################
QCP::QCP(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QCP"+std::to_string(id));
  progressMadeTowardsGoal = true;
  SetMetric("shortestpath");
}

void QCP::setup()
{
  BaseT::setup();
}

void QCP::clear()
{
  pdf_connectivity_configurations.clear();
  progressMadeTowardsGoal = true;
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

  int N = boost::num_edges(graph);
  if(!hasSolution){
    GrowWithoutSolution(ptc);
  }else{
    GrowWithSolution(ptc);
  }
  //DEBUG
  int M = boost::num_edges(graph);
  if( abs(N-M) > 100 ){
    std::cout << "increase of edges from " << N << " to " << M << std::endl;
    //exit(0);
  }
}

void QCP::GrowWithoutSolution(ob::PlannerTerminationCondition &ptc)
{
  if(NearestToGoalHasChanged() || progressMadeTowardsGoal)
  {
    //############################################################################
    //STATE1: GoalOriented Strategy
    //############################################################################
    Configuration* q_nearest = PriorityQueueNearestToGoal_Top();
    if(q_nearest == nullptr) return;

    std::cout << "GOTO GOAL" << std::endl;
    q_nearest->number_attempted_expansions++;
    progressMadeTowardsGoal = step_strategy->Towards(q_nearest, GetGoalConfiguration());
  }else{
    Configuration* q = PriorityQueueCandidate_PopTop();
    if(q!=nullptr){
      std::cout << "EXPAND OUTSIDE" << std::endl;
      //############################################################################
      //STATE2: ExtendFreeSpace Strategy (Active Node Expansion)
      //############################################################################
      if(q->index < 0){
        AddConfigurationToCover(q);
      }
      step_strategy->ExpandOutside(q);
    }else{
      std::cout << "EXPAND VORONOI" << std::endl;
      //############################################################################
      //STATE3: FindNewWays (Passive Node Expansion)
      //############################################################################
      //q = pdf_connectivity_configurations.sample(rng_.uniform01());
      //step_strategy->ExpandRandom(q);
      step_strategy->ExpandVoronoi();
    }
  }
}

void QCP::GrowWithSolution(ob::PlannerTerminationCondition &ptc)
{
  double r = rng_.uniform01();
  if(r <= rewireBias){
    RewireCover(ptc);
  }else{
    Configuration *q = pdf_connectivity_configurations.sample(rng_.uniform01());
    step_strategy->ExpandOutside(q);
    //step_strategy->ExpandRandom(q);
    pdf_connectivity_configurations.update(static_cast<PDF_Element*>(q->GetConnectivityPDFElement()), ValueConnectivity(q));
  }
}

//############################################################################
// Misc Functions
//############################################################################

double QCP::ValueConnectivity(Configuration *q)
{
  //Vertex v = get(indexToVertex, q->index);
  //QuotientCover::Print(q, false);
  double d_alpha = q->number_attempted_expansions;
  //double d_alpha = boost::degree(v, graph)+1;
  //double d_connectivity = q->GetRadius()/d_alpha;
  double d_connectivity = 1.0/(d_alpha+1);
  return d_connectivity;
}

void QCP::RewireCover(ob::PlannerTerminationCondition &ptc)
{
  Configuration *q = pdf_connectivity_configurations.sample(rng_.uniform01());

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
  nearest_neighborhood->nearestK(q, K, neighbors);

  if(neighbors.size()>=K){
    Configuration *qn = neighbors.at(K-1);
    double dn = GetMetric()->DistanceNeighborhoodNeighborhood(q, qn);
    if(dn <= 1e-10){
      AddEdge(q, qn);
      pdf_connectivity_configurations.update(static_cast<PDF_Element*>(qn->GetConnectivityPDFElement()), ValueConnectivity(qn));
      pdf_connectivity_configurations.update(static_cast<PDF_Element*>(q->GetConnectivityPDFElement()), ValueConnectivity(q));
    }
  }
}

QuotientCover::Vertex QCP::AddConfigurationToCover(Configuration *q)
{
  QuotientCover::Vertex v = BaseT::AddConfigurationToCover(q);

  PDF_Element *q_element = pdf_connectivity_configurations.add(q, ValueConnectivity(q));
  q->SetConnectivityPDFElement(q_element);

  return v;
}
void QCP::RemoveConfigurationFromCover(Configuration *q)
{
  pdf_connectivity_configurations.remove(static_cast<PDF_Element*>(q->GetConnectivityPDFElement()));
  BaseT::RemoveConfigurationFromCover(q);
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
