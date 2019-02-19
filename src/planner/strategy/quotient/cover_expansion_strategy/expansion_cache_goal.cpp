#include "expansion_cache_goal.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
#include <ompl/util/Time.h>

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyCacheGoal::CoverExpansionStrategyCacheGoal(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategyCache(quotient_cover_queue_)
{
}
double CoverExpansionStrategyCacheGoal::Step()
{
  if(quotient_cover_queue->GetParent()==nullptr){
    //Default strategy to go to goal
    q_source = quotient_cover_queue->PriorityQueueNearestToGoal_Top();
    if(q_source == nullptr) return -1;
    q_source->number_attempted_expansions++;
    progressMadeTowardsGoal = Towards(q_source, quotient_cover_queue->GetGoalConfiguration());
    if(progressMadeTowardsGoal) return 1;
    else return -1;
  }
    
  if(cached_path.empty())
  {
    std::cout << "Towards Goal from cached path" << std::endl;
    q_source = quotient_cover_queue->PriorityQueueNearestToGoal_Top();
    if(q_source == nullptr) return -1;
    CachePath(q_source, quotient_cover_queue->GetGoalConfiguration());
    q_milestone = new Configuration(quotient_cover_queue->GetQ1());
    RollUpPath();
    progressMadeTowardsGoal = Towards(q_source, q_milestone);
  }else{
    //Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q_source);
    //if(q_outward == nullptr) return -1;
    //progressMadeTowardsGoal = ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_source, q_outward);
    q_source = quotient_cover_queue->PriorityQueueNearestToGoal_Top();
    progressMadeTowardsGoal = Towards(q_source, quotient_cover_queue->GetGoalConfiguration());
  }
  q_source = const_cast<Configuration*>(q_last_expanded);
  //double t_2 = ompl::time::seconds(ompl::time::now() - t_start);

  //bool progressMadeTowardsGoal = ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_source, q_proj);
  //cached_path.at(0) = const_cast<Configuration*>(q_last_expanded);

  //std::cout << "Iteration t1=" << t_1 << " t2=" << t_2-t_1 << std::endl;
  if(progressMadeTowardsGoal) return 1;
  else return -1;
}
