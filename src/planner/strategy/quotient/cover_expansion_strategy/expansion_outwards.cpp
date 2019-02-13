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
  std::cout << "STEP OUTWARDS" << std::endl;
  Configuration* q = quotient_cover_queue->PriorityQueueCandidate_PopTop();
  if(q==nullptr) return -1;
  if(q->index < 0){
    quotient_cover_queue->AddConfigurationToCover(q);
  }
  Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q);
  if(q_outward == nullptr) return -1;
  else return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q, q_outward);
}

