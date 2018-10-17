#include "common.h"
#include "qng_goal_directed.h"

using namespace ompl::geometric;

QNGGoalDirected::QNGGoalDirected(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNGGoalDirected"+std::to_string(id));
}

QNGGoalDirected::~QNGGoalDirected(void)
{
  clear();
}

void QNGGoalDirected::clear()
{
  BaseT::clear();

  while(!configurations_sorted_by_nearest_to_goal.empty()) 
  {
    configurations_sorted_by_nearest_to_goal.pop();
  }
  if(q_start) configurations_sorted_by_nearest_to_goal.push(q_start);
  progressMadeTowardsGoal = true;
}
void QNGGoalDirected::setup()
{
  BaseT::setup();
  if(setup_){
    while(!configurations_sorted_by_nearest_to_goal.empty()) 
    {
      configurations_sorted_by_nearest_to_goal.pop();
    }
    configurations_sorted_by_nearest_to_goal.push(q_start);
  }
  progressMadeTowardsGoal = true;
}


void QNGGoalDirected::Grow(double t)
{

  //############################################################################
  //QNGGoalDirected: Two phase expansion planner
  //############################################################################

  if(saturated) return;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );

  //############################################################################
  //Phase1: No Solution has been found
  //############################################################################
  //
  //strategy: With probability 'goaldirectionadaptivebias' move towards goal. If
  //progress is made, increase goaldirectionadaptivebias. If no progress is
  //made, decrease. With probability '1.0 - goaldirectionadaptivebias' expand
  //towards free space. The rational here is that whenever we can 'see' the
  //goal, then we should move straight towards it. But whenever we are lost, we
  //should expand quickly to capture the free space. The expansion towards free
  //space biases the search towards larger neighborhoods, which is a modified
  //voronoi bias, but restricted to the free space only.
  //
  //############################################################################
  //Phase2: Solution Found
  //############################################################################
  //
  //strategy: Expand towards free space only. 

  if(!hasSolution){
    if(progressMadeTowardsGoal){
      progressMadeTowardsGoal = ExpandTowardsGoal(ptc);
    }else{
      double r = rng_.uniform01();
      if(r <= goalDirectionBias){
        progressMadeTowardsGoal = ExpandTowardsGoal(ptc);
      }else{
        ExpandTowardsFreeSpace(ptc);
      }
    }
  }else{
    ExpandTowardsFreeSpace(ptc);
  }
}

bool QNGGoalDirected::ExpandTowardsGoal(ob::PlannerTerminationCondition &ptc)
{
  //is it possible that we add here a state with zero-measure NBH?
  Configuration *q_nearest = configurations_sorted_by_nearest_to_goal.top();
  const double goalDistance = q_nearest->GetGoalDistance();
  if(goalDistance < 1e-10){
    hasSolution = true;
    return true;
  }
  return SteerTowards(q_nearest, q_goal);
}

bool QNGGoalDirected::SteerTowards(Configuration *q_from, Configuration *q_next)
{
  if(verbose>1)
  {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "from:" << std::endl;
    QuotientCover::Print(q_from);
    std::cout << "steer towards:" << std::endl;
    QuotientCover::Print(q_next);
  }

  const double radius_from = q_from->GetRadius();
  if(radius_from > thresholdObstaclesHorizon)
  {
    //move directly towards q_next
    Configuration *q_proj = new Configuration(Q1);
    Interpolate(q_from, q_next, q_proj);
    bool isProjectedFeasible = ComputeNeighborhood(q_proj);
    return isProjectedFeasible;

  }else{

    std::vector<Configuration*> q_children = GenerateCandidateDirections(q_from, q_next);

    //############################################################################
    //get best child: here best means the one with the largest radius, but this
    //might not be the best choice
    //############################################################################
    double radius_best = 0;
    uint idx_best = 0;
    for(uint k = 0; k < q_children.size(); k++)
    {
      Configuration *q_k = q_children.at(k);
      double r = q_k->GetRadius();
      if(r > radius_best)
      {
        radius_best = r;
        idx_best = k;
      }
      q_k->parent_neighbor = q_from;
    }


    //############################################################################
    //when to declare no progress made
    //############################################################################
    if(q_children.empty()){
      return false;
    }

    Configuration *q_best = q_children.at(idx_best);
    AddConfigurationToCoverWithoutAddingEdges(q_best);
    configurations_sorted_by_nearest_to_goal.push(q_best);

    //add the smaller children to the priority_configurations to be extended at
    //a later stage if required
    for(uint k = 0; k < q_children.size(); k++)
    {
      Configuration *q_k = q_children.at(k);
      if(k!=idx_best){
        priority_configurations.push(q_k);
      }
    }
  }
  return true;
}

std::vector<QuotientCover::Configuration*> QNGGoalDirected::GenerateCandidateDirections(Configuration *q_from, Configuration *q_next)
{
  std::vector<Configuration*> q_children;

  //            \                                           |
  //             \q_k                                       |
  //              \                                         |
  // q_from        |q_proj                      q_next      |
  //              /                                         |
  //             /                                          |
  Configuration *q_proj = new Configuration(Q1);
  const double radius_from = q_from->GetRadius();
  double d = DistanceConfigurationConfiguration(q_from, q_next);
  Q1->getStateSpace()->interpolate(q_from->state, q_next->state, radius_from/d, q_proj->state);

  bool isProjectedFeasible = ComputeNeighborhood(q_proj);

  if(isProjectedFeasible)
  {
    q_proj->parent_neighbor = q_from;
    q_children.push_back(q_proj);
    double radius_proj = q_proj->GetRadius();
    //terminate if projected moves towards larger neighborhoods
    if(radius_proj >= radius_from){
      return q_children;
    }
  }else{
    return q_children;
  }
  for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES; k++){
    Configuration *q_k = new Configuration(Q1);

    Q1_sampler->sampleUniformNear(q_k->state, q_proj->state, 0.5*radius_from);

    const double d_from_to_k = DistanceConfigurationConfiguration(q_from, q_k);
    Q1->getStateSpace()->interpolate(q_from->state, q_k->state, radius_from/d_from_to_k, q_k->state);

    if(ComputeNeighborhood(q_k))
    {
      q_k->parent_neighbor = q_from;
      q_children.push_back(q_k);
    }
  }
  return q_children;
}


void QNGGoalDirected::ExpandTowardsFreeSpace(ob::PlannerTerminationCondition &ptc)
{
  if(priority_configurations.empty()){
    //try random directions
    //saturated = true;
    if(verbose>0) std::cout << "Space got saturated." << std::endl;
    Configuration *q_random = SampleCoverBoundaryValid(ptc);
    if(q_random == nullptr) return;
    priority_configurations.push(q_random);
  }

  Configuration *q = priority_configurations.top();
  priority_configurations.pop();
  if(IsConfigurationInsideCover(q)){
    q->Remove(Q1);
    q=nullptr;
    return;
  }
  AddConfigurationToCoverWithoutAddingEdges(q);
  configurations_sorted_by_nearest_to_goal.push(q);

  if(verbose>0) QuotientCover::Print(q);

  std::vector<Configuration*> q_children = ExpandNeighborhood(q, NUMBER_OF_EXPANSION_SAMPLES);
  for(uint k = 0; k < q_children.size(); k++)
  {
    priority_configurations.push(q_children.at(k));
  }
  //add different biases to remove planner from getting stuck
  Configuration *q_random = QuotientCover::SampleCoverBoundary("voronoi");
  if(ComputeNeighborhood(q_random)){
    priority_configurations.push(q_random);
  }
}
