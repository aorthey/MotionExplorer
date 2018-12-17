#include "step_straight.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

bool StepStrategyStraight::Towards(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to)
{
  const double distanceFromTo = quotient_cover_queue->DistanceNeighborhoodNeighborhood(q_from, q_to);
  if(distanceFromTo < 1e-10){
    return false;
  }
  std::vector<Configuration*> q_children = GenerateCandidateDirections(q_from, q_to);

  if(q_children.empty()){
    return false;
  }

  uint idx_best = quotient_cover_queue->GetLargestNeighborhoodIndex(q_children);

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

bool StepStrategyStraight::ConfigurationHasNeighborhoodLargerThan(QuotientCover::Configuration *q, double radius)
{
  return (q->GetRadius() >= radius);
}

std::vector<QuotientCover::Configuration*> StepStrategyStraight::GenerateCandidateDirections(Configuration *q_from, Configuration *q_to)
{
  std::vector<Configuration*> q_children;

  //            \                                           |
  //             \q_k                                       |
  //              \                                         |
  // q_from        |q_proj                      q_to      |
  //              /                                         |
  //             /                                          |
  //############################################################################
  //Case(1): Next projected onto neighborhood has larger radius => expand
  //directly, larger is always better
  //############################################################################
  Configuration *q_proj = new Configuration(quotient_cover_queue->GetQ1());

  //const double radius_from = q_from->GetRadius();
  quotient_cover_queue->Connect(q_from, q_to, q_proj);
  q_proj->parent_neighbor = q_from;

  if(quotient_cover_queue->ComputeNeighborhood(q_proj)){
    q_children.push_back(q_proj);
  }

  return q_children;

  //############################################################################
  //Case(2): Next projected is smaller
  //############################################################################
  //if(q_from->parent_neighbor != nullptr){
  //  Configuration *q_sample_direction = q_proj;
  //  Configuration *q_momentum = new Configuration(Q1);

  //  double radius_parent = q_from->parent_neighbor->GetRadius();
  //  double step = (radius_from+radius_parent)/radius_parent;
  //  Interpolate(q_from->parent_neighbor, q_from, step, q_momentum);

  //  //CheckConfigurationIsOnBoundary(q_momentum, q_from);
  //  q_momentum->parent_neighbor = q_from;
  //  //############################################################################

  //  if(ComputeNeighborhood(q_momentum)){
  //    q_children.push_back(q_momentum);
  //    if(ConfigurationHasNeighborhoodLargerThan(q_momentum, radius_from))
  //    {
  //      return q_children;
  //    }
  //  }

  //  if(q_momentum->GetRadius() > q_proj->GetRadius())
  //  {
  //    q_sample_direction = q_momentum;
  //  }
  //}

  //############################################################################
  //Case(3): Neither proj nor momentum configuration are better. Try random
  //sampling
    //for(uint k = 0; k < quotient_cover_queue->NUMBER_OF_EXPANSION_SAMPLES; k++){
  //  Configuration *q_k = new Configuration(quotient_cover_queue->Q1);

  //  quotient_cover_queue->Q1_sampler->sampleUniformNear(q_k->state, q_sample_direction->state, 0.5*radius_from);

  //  const double d_from_to_k = quotient_cover_queue->DistanceConfigurationConfiguration(q_from, q_k);

  //  double step_size = radius_from/d_from_to_k;
    
  //  quotient_cover_queue->Q1->getStateSpace()->interpolate(q_from->state, q_k->state, step_size, q_k->state);

  //  // std::cout << "radius q_from: " << q_from->GetRadius() << std::endl;
  //  // std::cout << "d_from_to_k  : " << d_from_to_k << std::endl;
  //  // std::cout << "step         : " << step_size << std::endl;
  //  // std::cout << "d_from_to_k  (after proj): " << DistanceQ1(q_from, q_k) << std::endl;
  //  // double d_outcome = DistanceConfigurationConfiguration(q_from, q_k);
  //  // std::cout << "d_from_to_k  (graph metric): " << d_outcome << std::endl;

  //  //PROBLEM: q_k does not have a coset after sampling. Therefore we utilize
  //  //internally DistanceQ1. After Computeneighborhood, the coset is set, and we
  //  //can actually utilize the graph metric. However, under the graph metric,
  //  //the distance changes and it is not on the boundary anymore. Not sure how
  //  //to resolve
  //  //
  //  //
  //  if(quotient_cover_queue->ComputeNeighborhood(q_k))
  //  {
  //    q_k->parent_neighbor = q_from;
  //    q_children.push_back(q_k);
  //  }
  //}
  return q_children;
}
