#include "step.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

StepStrategy::StepStrategy()
{
}

void StepStrategy::SetSpace(QuotientCoverQueue *quotient_cover_queue_)
{
  quotient_cover_queue = quotient_cover_queue_;
}


bool StepStrategy::ExpandOutside(QuotientCover::Configuration *q_from)
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

bool StepStrategy::ExpandRandom(QuotientCover::Configuration *q_from)
{
  Configuration *q_next = new Configuration(quotient_cover_queue->GetQ1());

  // q_from --------- q_next on boundary of NBH of q_from
  quotient_cover_queue->GetQ1SamplerPtr()->sampleUniformNear(q_next->state, q_from->state, q_from->GetRadius());
  double d = quotient_cover_queue->GetQ1()->distance(q_next->state, q_from->state);
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
