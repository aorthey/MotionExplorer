#include "expansion_goal.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyGoal::CoverExpansionStrategyGoal(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategy(quotient_cover_queue_)
{
}
double CoverExpansionStrategyGoal::Step()
{
  Configuration* q_nearest = quotient_cover_queue->PriorityQueueNearestToGoal_Top();
  if(q_nearest == nullptr) return -1;
  q_nearest->number_attempted_expansions++;
  bool progressMadeTowardsGoal = Towards(q_nearest, quotient_cover_queue->GetGoalConfiguration());
  if(progressMadeTowardsGoal) return 1;
  else return -1;
}
