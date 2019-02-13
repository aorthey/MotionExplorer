#include "expansion_random_boundary.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyRandomBoundary::CoverExpansionStrategyRandomBoundary(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategy(quotient_cover_queue_)
{
}
double CoverExpansionStrategyRandomBoundary::Step()
{
  Configuration *q = quotient_cover_queue->GetConfigurationLowConnectivity();
  bool expansionSuccessful = ExpandTowardsSteepestAscentDirectionFromInitialDirection(q, nullptr);
  if(expansionSuccessful){
    quotient_cover_queue->PDFConnectivityUpdate(q);
    return +1;
  }else{
    return -1;
  }
}

