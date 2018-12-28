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
bool StepStrategyStraight::Expand(QuotientCover::Configuration *q_from)
{
  if(q_from->parent_neighbor != nullptr){
    Configuration *q_next = new Configuration(quotient_cover_queue->GetQ1());

    // q_parent ------- q_from --------- q_next
    double radius_from = q_from->GetRadius();
    double radius_parent = q_from->parent_neighbor->GetRadius();
    double step = (radius_from+radius_parent)/radius_parent;

    quotient_cover_queue->Interpolate(q_from->parent_neighbor, q_from, step, q_next);

    q_next->parent_neighbor = q_from;
    //############################################################################

    if(quotient_cover_queue->ComputeNeighborhood(q_next)){
      quotient_cover_queue->AddConfigurationToCover(q_next);
      return true;
    }
  }else{
    std::cout << "tried to expand start configuration straight. Not defined." << std::endl;
    exit(0);
  }
  return false;
}
