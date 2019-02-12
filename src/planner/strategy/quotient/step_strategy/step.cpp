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
  double d = quotient_cover_queue->GetMetric()->DistanceConfigurationConfiguration(q_next, q_from);
  quotient_cover_queue->GetQ1()->getStateSpace()->interpolate(q_from->state, q_next->state, q_from->GetRadius()/d, q_next->state);

  //q_next->parent_neighbor = q_from;
  //############################################################################

  if(quotient_cover_queue->ComputeNeighborhood(q_next)){
    quotient_cover_queue->AddConfigurationToCover(q_next);
    //quotient_cover_queue->AddConfigurationToPriorityQueue(q_k);
    return true;
  }
  return false;
}
bool StepStrategy::ExpandVoronoi()
{
  // q_from --------- q_next on boundary of NBH of q_from
  //
  const ob::SpaceInformationPtr &Q1 = quotient_cover_queue->GetQ1();
  Configuration *q_random = new Configuration(Q1);
  quotient_cover_queue->SampleUniform(q_random);

  Configuration *q_nearest = quotient_cover_queue->NearestNeighborhood(q_random);
  quotient_cover_queue->Print(q_random, false);
  quotient_cover_queue->Print(q_nearest, false);

  double d = quotient_cover_queue->GetMetric()->DistanceQ1(q_nearest, q_random);

  Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, q_nearest->GetRadius()/d, q_random->state);

  if(quotient_cover_queue->ComputeNeighborhood(q_random)){
    quotient_cover_queue->AddConfigurationToCover(q_random);
    return true;
  }
  return false;
}
