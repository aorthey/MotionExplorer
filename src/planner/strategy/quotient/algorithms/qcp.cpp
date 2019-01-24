#include "common.h"
#include "gui/common.h"
#include "qcp.h"
#include "planner/strategy/quotient/step_strategy/step_straight.h"
#include "planner/strategy/quotient/step_strategy/step_adaptive.h"

using namespace ompl::geometric;

//############################################################################
// Setup
//############################################################################
QCP::QCP(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QCP"+std::to_string(id));
  progressMadeTowardsGoal = true;
  step_strategy = new StepStrategyAdaptive();
  step_strategy->SetSpace(this);
}

QCP::~QCP(void)
{
}

void QCP::setup()
{
  BaseT::setup();
}

void QCP::clear()
{
  pdf_connectivity_configurations.clear();
  while(!configurations_sorted_by_nearest_to_goal.empty()) 
  {
    configurations_sorted_by_nearest_to_goal.pop();
  }

  nearest_to_goal_has_changed = true;
  progressMadeTowardsGoal = true;
  firstRun = true;
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
    AddConfigurationToPriorityQueue(q_start);
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
  if(nearest_to_goal_has_changed || progressMadeTowardsGoal)
  {
    nearest_to_goal_has_changed = false;
    //############################################################################
    //STATE1: GoalOriented Strategy
    //############################################################################

    if(verbose>1) std::cout << "Step Towards Goal" << std::endl;
    Configuration *q_nearest = configurations_sorted_by_nearest_to_goal.top();
    progressMadeTowardsGoal = step_strategy->Towards(q_nearest, q_goal);
    if(!progressMadeTowardsGoal){
      q_nearest->number_attempted_expansions++;
    }
    if(verbose>1) std::cout << "Progress: "<< (progressMadeTowardsGoal?"Yes":"No") << std::endl;

    //############################################################################
  }else{
    if(!priority_queue_candidate_configurations.empty()){
      if(verbose>1) std::cout << std::string(80, '-') << std::endl;
      if(verbose>1) PrintQueue(10);
      //############################################################################
      //STATE2: ExtendFreeSpace Strategy (Active Node Expansion)
      //############################################################################
      if(verbose>1) std::cout << "Expand Largest Single-Connected Node" << std::endl;

      Configuration *q = priority_queue_candidate_configurations.top();
      priority_queue_candidate_configurations.pop();
      if(q->index < 0){
        AddConfigurationToCover(q);
      }
      step_strategy->ExpandOutside(q);
      if(verbose>1) PrintQueue(10);
      if(verbose>1) std::cout << std::string(80, '-') << std::endl;
    }else{
      //############################################################################
      //STATE3: FindNewWays (Passive Node Expansion)
      //############################################################################
      if(verbose>1) std::cout << "Generate New Configurations" << std::endl;
      Configuration *q = pdf_connectivity_configurations.sample(rng_.uniform01());
      step_strategy->ExpandRandom(q);
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
    step_strategy->ExpandRandom(q);
  }
}

//############################################################################
// Misc Functions
//############################################################################

double QCP::ValueConnectivity(Configuration *q)
{
  //Vertex v = get(indexToVertex, q->index);
  //QuotientCover::Print(q, false);
  //double d_alpha = std::pow(2.0,boost::degree(v, graph));
  //double d_alpha = boost::degree(v, graph)+1;
  //double d_connectivity = q->GetRadius()/d_alpha;
  double d_connectivity = q->GetRadius();
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

  //add one more edges
  std::vector<Configuration*> neighbors;
  Vertex v = get(indexToVertex, q->index);
  uint K = boost::degree(v, graph)+1;
  nearest_neighborhood->nearestK(q, K, neighbors);

  if(neighbors.size()>=K){
    Configuration *qn = neighbors.at(K-1);
    double dn = DistanceNeighborhoodNeighborhood(q, qn);
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

  Configuration *q_nearest = nullptr;
  if(!configurations_sorted_by_nearest_to_goal.empty()){
    q_nearest = configurations_sorted_by_nearest_to_goal.top();
  }

  configurations_sorted_by_nearest_to_goal.push(q);
  if(q_nearest != configurations_sorted_by_nearest_to_goal.top()){
    //change in nearest towards goal. This should trigger the step towards goal
    //method. This removes the goalBias
    nearest_to_goal_has_changed = true;
  }

  return v;
}

