#include "step_adaptive.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

StepStrategyAdaptive::StepStrategyAdaptive(QuotientCoverQueue *quotient_cover_queue_):
  StepStrategy(quotient_cover_queue_)
{
}
bool StepStrategyAdaptive::ExpandOutside(QuotientCover::Configuration *q_from)
{
  Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q_from);
  if(q_outward == nullptr) return false;
  else return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_outward);
}

bool StepStrategyAdaptive::ExpandRandom(QuotientCover::Configuration *q_from)
{
  return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, nullptr);
}
bool StepStrategyAdaptive::ExpandVoronoi()
{
  const ob::SpaceInformationPtr &Q1 = quotient_cover_queue->GetQ1();
  Configuration *q_random = new Configuration(Q1);
  quotient_cover_queue->SampleUniform(q_random);

  Configuration *q_nearest = quotient_cover_queue->NearestNeighborhood(q_random);

  return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_nearest, q_random);
}

