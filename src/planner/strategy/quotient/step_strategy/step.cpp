#include "step.h"
using namespace ompl::geometric;

StepStrategy::StepStrategy()
{
}

void StepStrategy::SetSpace(QuotientCoverQueue *quotient_cover_queue_)
{
  quotient_cover_queue = quotient_cover_queue_;
}


