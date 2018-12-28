#include "step_adaptive.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

bool StepStrategyAdaptive::Towards(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to)
{
  const double distanceFromTo = quotient_cover_queue->DistanceNeighborhoodNeighborhood(q_from, q_to);
  if(distanceFromTo < 1e-10){
    return false;
  }
  std::vector<Configuration*> q_children = GenerateCandidateDirections(q_from, q_to);
  return ChooseBestDirection(q_children);
}

bool StepStrategyAdaptive::Expand(QuotientCover::Configuration *q_from)
{
  if(q_from->parent_neighbor != nullptr){

    Configuration *q_next = new Configuration(quotient_cover_queue->GetQ1());

    //############################################################################
    //Generate q_next in the direction away from parent
    // q_parent ------- q_from --------- q_next
    //############################################################################
    double radius_from = q_from->GetRadius();
    double radius_parent = q_from->parent_neighbor->GetRadius();
    double step = (radius_from+radius_parent)/radius_parent;

    quotient_cover_queue->Interpolate(q_from->parent_neighbor, q_from, step, q_next);

    q_next->parent_neighbor = q_from;

    //############################################################################
    //Addconfigurationrandomperturbation
    //############################################################################
    std::vector<Configuration*> q_children;
    if(quotient_cover_queue->ComputeNeighborhood(q_next)){
      q_children.push_back(q_next);
      if(q_next->GetRadius() >= radius_from){
        //return if we go towards larger area
        return ChooseBestDirection(q_children);
      }
    }else{
      //no progress made
      return false;
    }
    GenerateRandomChildrenAroundConfiguration(q_next, q_children, radius_from);
    return ChooseBestDirection(q_children);
  }else{
    std::cout << "tried to expand start configuration straight. Not defined." << std::endl;
    exit(0);
    return false;
  }
}

bool StepStrategyAdaptive::ChooseBestDirection(const std::vector<Configuration*> &q_children)
{
  if(q_children.empty()){
    return false;
  }
  uint idx_best = GetLargestNeighborhoodIndex(q_children);

  Configuration *q_best = q_children.at(idx_best);
  quotient_cover_queue->AddConfigurationToCover(q_best);

  //add the smaller children to the priority_configurations to be extended at
  //a later stage if required
  for(uint k = 0; k < q_children.size(); k++)
  {
    Configuration *q_k = q_children.at(k);
    if(k!=idx_best){
      quotient_cover_queue->AddConfigurationToPriorityQueue(q_k);
    }
  }
  return true;
}

uint StepStrategyAdaptive::GetLargestNeighborhoodIndex(const std::vector<QuotientCover::Configuration*> &q_children)
{
  double radius_best = 0;
  uint idx_best = 0;
  for(uint k = 0; k < q_children.size(); k++)
  {
    Configuration *q_k = q_children.at(k);
    double r = q_k->GetRadius();
    if(r > radius_best)
    {
      radius_best = r;
      idx_best = k;
    }
  }
  return idx_best;
}


bool StepStrategyAdaptive::ConfigurationHasNeighborhoodLargerThan(QuotientCover::Configuration *q, double radius)
{
  return (q->GetRadius() >= radius);
}

std::vector<QuotientCover::Configuration*> StepStrategyAdaptive::GenerateCandidateDirections(Configuration *q_from, Configuration *q_to)
{
  std::vector<Configuration*> q_children;

  //            \                                           |
  //             \q_k                                       |
  //              \                                         |
  // q_from        |q_proj                      q_to      |
  //              /                                         |
  //             /                                          |
  //############################################################################
  //Case(1): Projected Configuration (PC) onto neighborhood has larger radius => expand
  //directly, larger is always better
  //############################################################################
  Configuration *q_proj = new Configuration(quotient_cover_queue->GetQ1());

  const double radius_from = q_from->GetRadius();
  quotient_cover_queue->Interpolate(q_from, q_to, q_proj);
  q_proj->parent_neighbor = q_from;

  if(quotient_cover_queue->ComputeNeighborhood(q_proj)){
    q_children.push_back(q_proj);
    if(q_proj->GetRadius() >= radius_from){
      //return if we go towards larger area
      return q_children;
    }
  }else{
    //no progress made
    return q_children;
  }

  //############################################################################
  //Case(2): Try random
  //sampling near PC, but on boundary of q_from
  //############################################################################

  GenerateRandomChildrenAroundConfiguration(q_proj, q_children, radius_from);
  return q_children;
}

void StepStrategyAdaptive::GenerateRandomChildrenAroundConfiguration(Configuration *q, std::vector<Configuration*> &q_children, double radius_from)
{
  uint NUMBER_OF_EXPANSION_SAMPLES = (quotient_cover_queue->GetQ1()->getStateDimension()+2);

  for(uint k = 0; k < NUMBER_OF_EXPANSION_SAMPLES; k++){
    Configuration *q_k = new Configuration(quotient_cover_queue->GetQ1());

    //Sample a configuration on the boundary of q_from, randomly distributed
    //around PC
    quotient_cover_queue->GetQ1SamplerPtr()->sampleGaussian(q_k->state, q->state, radius_from);
    const double d_from_to_k = quotient_cover_queue->DistanceConfigurationConfiguration(q, q_k);
    double step_size = radius_from/d_from_to_k;
    quotient_cover_queue->GetQ1()->getStateSpace()->interpolate(q->state, q_k->state, step_size, q_k->state);

    if(quotient_cover_queue->ComputeNeighborhood(q_k))
    {
      q_k->parent_neighbor = q;
      q_children.push_back(q_k);
    }
  }
}
