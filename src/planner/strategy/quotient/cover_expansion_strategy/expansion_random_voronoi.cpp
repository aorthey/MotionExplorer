#include "expansion_random_voronoi.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyRandomVoronoi::CoverExpansionStrategyRandomVoronoi(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategy(quotient_cover_queue_)
{
}
double CoverExpansionStrategyRandomVoronoi::Step()
{
  std::cout << "STEP RANDOM VORONOI" << std::endl;
  const ob::SpaceInformationPtr &Q1 = quotient_cover_queue->GetQ1();
  Configuration *q_random = new Configuration(Q1);
  quotient_cover_queue->SampleUniform(q_random);

  Configuration *q_nearest = quotient_cover_queue->NearestNeighborhood(q_random);

  return ExpandTowardsSteepestAscentDirectionFromInitialDirection(q_nearest, q_random);
}

