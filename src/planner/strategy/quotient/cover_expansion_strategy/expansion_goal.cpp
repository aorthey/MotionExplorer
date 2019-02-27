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
  if(q_target == nullptr){
    q_target = quotient_cover_queue->PriorityQueueNearestToGoal_Top();
    if(q_target == nullptr) return -1;
  }
  quotient_cover_queue->Print(q_target, false);
  q_target->number_attempted_expansions++;
  bool progressMadeTowardsGoal = Towards(q_target, quotient_cover_queue->GetGoalConfiguration());
  q_target = q_last_expanded;
  if(progressMadeTowardsGoal) return 1;
  else return -1;
}

void CoverExpansionStrategyGoal::Clear()
{
  BaseT::Clear();
  q_target = nullptr;
}
