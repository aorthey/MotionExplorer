#include "step.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

StepStrategy::StepStrategy(QuotientCoverQueue *quotient_cover_queue_):
  quotient_cover_queue(quotient_cover_queue_)
{
}


bool StepStrategy::ExpandOutside(QuotientCover::Configuration *q_from)
{
  std::cout << "NYI" << std::endl;
  exit(0);
}

bool StepStrategy::ExpandRandom(QuotientCover::Configuration *q_from)
{
  // q_from --------- q_next on boundary of NBH of q_from
  //
  Configuration *q_next = quotient_cover_queue->SampleNeighborhoodBoundary(q_from);

  quotient_cover_queue->GetQ1SamplerPtr()->sampleUniformNear(q_next->state, q_from->state, q_from->GetRadius());
  double d = metric->DistanceConfigurationConfiguration(q_next, q_from);
  quotient_cover_queue->GetQ1()->getStateSpace()->interpolate(q_from->state, q_next->state, q_from->GetRadius()/d, q_next->state);

  q_next->parent_neighbor = q_from;
  //############################################################################

  if(quotient_cover_queue->ComputeNeighborhood(q_next)){
    quotient_cover_queue->AddConfigurationToCover(q_next);
    //quotient_cover_queue->AddConfigurationToPriorityQueue(q_k);
    return true;
  }
  return false;
}
