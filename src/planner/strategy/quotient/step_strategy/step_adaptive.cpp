#include "step_adaptive.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

StepStrategyAdaptive::StepStrategyAdaptive(QuotientCoverQueue *quotient_cover_queue_):
  StepStrategy(quotient_cover_queue_)
{
}
bool StepStrategyAdaptive::Towards(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to)
{
  // const double distanceFromTo = quotient_cover_queue->GetMetric()->DistanceNeighborhoodNeighborhood(q_from, q_to);
  // if(distanceFromTo < 1e-10){
  //   return false;
  // }

  Configuration *q_proj = quotient_cover_queue->NearestConfigurationOnBoundary(q_from, q_to);
  if(q_from->isStart){
    return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_proj);
  }
  Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q_from);

  double r_proj = 0;
  double r_outward = 0;
  if(q_proj != nullptr) r_proj = q_proj->GetRadius();
  if(q_outward != nullptr) r_outward = q_outward->GetRadius();

  if(r_proj <= 0 && r_outward <= 0){
    return false;
  }else{
    if(r_proj > r_outward){
      return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_proj);
    }else{
      return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_outward);
    }
  }
}

bool StepStrategyAdaptive::ExpandOutside(QuotientCover::Configuration *q_from)
{
  Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q_from);
  if(q_outward == nullptr) return false;
  else return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_outward);
}

bool StepStrategyAdaptive::ExpandRandom(QuotientCover::Configuration *q_from)
{
  return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, nullptr);
}

bool StepStrategyAdaptive::ExpandTowardsSteepestAscentDirectionFromInitialDirection(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_initial)
{
  const double radius_from = q_from->GetRadius();
  double radius_sampling = 0.1*q_from->GetRadius();

  const uint M = (quotient_cover_queue->GetQ1()->getStateDimension()+2);

  uint ctr = 0;
  while(q_initial == nullptr && ctr++ <= M){
    q_initial = quotient_cover_queue->SampleNeighborhoodBoundary(q_from);
  }

  if(q_initial == nullptr){
    return false;
  }
  ctr = 0;

  double radius_last = q_initial->GetRadius();


  Configuration *q_last = q_initial;
  Configuration *q_next;
  while(true){
    if(radius_last >= radius_from){
      quotient_cover_queue->AddConfigurationToCover(q_last);
      quotient_cover_queue->AddConfigurationToPriorityQueue(q_last);
      return true;
    }else{
      q_next = quotient_cover_queue->SampleNeighborhoodBoundaryUniformNear(q_from, q_last, radius_sampling);
    }
    if(q_next != nullptr && q_next->GetRadius() > radius_last){
      q_last = q_next;
      radius_last = q_next->GetRadius();
      ctr = 0;
      radius_sampling = 0.1*q_from->GetRadius();
    }else{
      radius_sampling *= 2;
      ctr++;
    }
    //Terminate if we could not improve solution M times in a row
    if(ctr>M) break;
  }
  if(radius_last>0){
    quotient_cover_queue->AddConfigurationToCover(q_last);
    quotient_cover_queue->AddConfigurationToPriorityQueue(q_last);
    return true;
  }else{
    return false;
  }

}

bool StepStrategyAdaptive::ChooseBestDirectionOnlyAddBestToQueue(const std::vector<Configuration*> &q_children)
{
  if(q_children.empty()){
    return false;
  }

  uint idx_best = GetLargestNeighborhoodIndex(q_children);
  Configuration *q_best = q_children.at(idx_best);
  quotient_cover_queue->AddConfigurationToCover(q_best);
  quotient_cover_queue->AddConfigurationToPriorityQueue(q_best);
  return true;
}
bool StepStrategyAdaptive::ChooseBestDirection(const std::vector<Configuration*> &q_children, bool addBestToPriorityQueue)
{
  if(q_children.empty()){
    return false;
  }

  uint idx_best = GetLargestNeighborhoodIndex(q_children);
  Configuration *q_best = q_children.at(idx_best);

  if(verbose>1) std::cout << "COVER add NBH with radius " << q_best->GetRadius() << std::endl;
  quotient_cover_queue->AddConfigurationToCover(q_best);

  //add the smaller children to the priority_configurations to be extended at
  //a later stage if required
  for(uint k = 0; k < q_children.size(); k++)
  {
    Configuration *q_k = q_children.at(k);
    if(addBestToPriorityQueue){
      quotient_cover_queue->AddConfigurationToPriorityQueue(q_k);
    }else{
      if(k!=idx_best){
        quotient_cover_queue->AddConfigurationToPriorityQueue(q_k);
      }
    }
  }
  return true;
}

uint StepStrategyAdaptive::GetLargestNeighborhoodIndexOutsideCover(const std::vector<QuotientCover::Configuration*> &q_children)
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

