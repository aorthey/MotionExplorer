#include "expansion.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
#include <ompl/util/Time.h>

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategy::CoverExpansionStrategy(QuotientCoverQueue *quotient_cover_queue_):
  quotient_cover_queue(quotient_cover_queue_)
{
}
void CoverExpansionStrategy::Clear()
{
  q_last_expanded = nullptr;
}

bool CoverExpansionStrategy::TowardsStraightLine(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to)
{
  const double distanceFromTo = quotient_cover_queue->GetMetric()->DistanceNeighborhoodNeighborhood(q_from, q_to);
  if(distanceFromTo < 1e-10){
    return false;
  }

  Configuration *q_proj = new Configuration(quotient_cover_queue->GetQ1());

  quotient_cover_queue->GetMetric()->Interpolate(q_from, q_to, q_proj);

  if(quotient_cover_queue->ComputeNeighborhood(q_proj)){
    quotient_cover_queue->AddConfigurationToCover(q_proj);
    q_last_expanded = q_proj;
    return true;
  }

  return false;
}

bool CoverExpansionStrategy::TowardsBoundaryPoint(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_boundary_point)
{
  //ompl::time::point t1 = ompl::time::now();

  Configuration* q_outward = quotient_cover_queue->GetOutwardPointingConfiguration(q_from);

  //ompl::time::point t2 = ompl::time::now();
  double r_boundary = 0;
  double r_outward = 0;
  if(q_boundary_point != nullptr) r_boundary = q_boundary_point->GetRadius();
  if(q_outward != nullptr) r_outward = q_outward->GetRadius();

  bool progress = false;
  if(r_boundary <= 0 && r_outward <= 0){
    return false;
  }else{
    if(r_boundary > r_outward){
      progress = ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_boundary_point);
    }else{
      progress = ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_outward);
    }
  }
  return progress;
}
bool CoverExpansionStrategy::Towards(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to)
{
  //ompl::time::point t_start = ompl::time::now();
  Configuration *q_proj = quotient_cover_queue->NearestConfigurationOnBoundary(q_from, q_to);
  if(q_from->isStart){
    return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_from, q_proj);
  }else{
    return TowardsBoundaryPoint(q_from, q_proj);
  }
}

bool CoverExpansionStrategy::ExpandTowardsSteepestAscentDirectionFromInitialDirection(const QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_initial)
{
  const double radius_from = q_from->GetRadius();
  double radius_sampling = 0.1*q_from->GetRadius();

  const uint M = (quotient_cover_queue->GetQ1()->getStateDimension()+10);

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
      q_last_expanded = q_last;
      return true;
    }else{
      q_next = quotient_cover_queue->SampleNeighborhoodBoundaryUniformNear(q_from, q_last, radius_sampling);
    }
    //greey jumps
    if(q_next != nullptr && q_next->GetRadius() > radius_last){
      q_last = q_next;
      radius_last = q_next->GetRadius();
      ctr = 0;
      radius_sampling = 0.5*q_from->GetRadius();
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
    q_last_expanded = q_last;
    return true;
  }else{
    return false;
  }

}
