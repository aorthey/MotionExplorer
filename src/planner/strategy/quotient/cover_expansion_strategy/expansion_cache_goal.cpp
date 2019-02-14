#include "expansion_cache_goal.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyCacheGoal::CoverExpansionStrategyCacheGoal(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategyCache(quotient_cover_queue_)
{
}
double CoverExpansionStrategyCacheGoal::Step()
{
  if(cached_path.empty())
  {
    q_source = quotient_cover_queue->PriorityQueueNearestToGoal_Top();
    if(q_source == nullptr) return -1;
    CachePath(q_source, quotient_cover_queue->GetGoalConfiguration());
  }

  //q_source = cached_path.at(0);
  //q_source->number_attempted_expansions++;

  RollUpPath(cached_path);

  Configuration *q_proj = new Configuration(quotient_cover_queue->GetQ1());

  bool progressMadeTowardsGoal = ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_source, q_proj);
  cached_path.at(0) = const_cast<Configuration*>(q_last_expanded);

  if(progressMadeTowardsGoal) return 1;
  else return -1;
}
