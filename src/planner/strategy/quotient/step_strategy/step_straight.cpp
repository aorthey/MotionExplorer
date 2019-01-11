#include "step_straight.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

bool StepStrategyStraight::Towards(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to)
{
  const double distanceFromTo = quotient_cover_queue->DistanceNeighborhoodNeighborhood(q_from, q_to);
  if(distanceFromTo < 1e-10){
    return false;
  }

  Configuration *q_proj = new Configuration(quotient_cover_queue->GetQ1());

  quotient_cover_queue->Connect(q_from, q_to, q_proj);
  q_proj->parent_neighbor = q_from;

  if(quotient_cover_queue->ComputeNeighborhood(q_proj)){
    quotient_cover_queue->AddConfigurationToCover(q_proj);
    return true;
  }

  return false;
}
