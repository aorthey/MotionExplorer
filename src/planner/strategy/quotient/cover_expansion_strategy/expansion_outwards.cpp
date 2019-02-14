#include "expansion_outwards.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyOutwards::CoverExpansionStrategyOutwards(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategy(quotient_cover_queue_)
{
}
double CoverExpansionStrategyOutwards::Step()
{
  Configuration* q = quotient_cover_queue->PriorityQueueCandidate_PopTop();
  if(q==nullptr){
    //no single-connected neighborhoods. Choose based on largest
    q = quotient_cover_queue->SampleConfigurationLargestNeighborhood();
  }
  if(q->index < 0){
    quotient_cover_queue->AddConfigurationToCover(q);
  }
  Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q);
  if(q_outward == nullptr) return -1;
  else return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q, q_outward);
}

