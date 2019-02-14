#include "expansion_cache.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
#include "planner/strategy/quotient/metric/quotient_metric_shortest_path.h"

using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

CoverExpansionStrategyCache::CoverExpansionStrategyCache(og::QuotientCoverQueue* quotient_cover_queue_):
  CoverExpansionStrategy(quotient_cover_queue_)
{
}
void CoverExpansionStrategyCache::CachePath(const Configuration *q_from, const Configuration *q_to)
{
  std::cout << "COMPUTING CACHED PATH" << std::endl;
  QuotientMetricShortestPathPtr metric = static_pointer_cast<QuotientMetricShortestPath>(quotient_cover_queue->GetMetric());
  cached_path = metric->GetInterpolationPath(q_from, q_to);
}

void CoverExpansionStrategyCache::RollUpPath(ConfigurationPath &path)
{
  const Configuration *q = path.at(0);
  Configuration *q_milestone = new Configuration(quotient_cover_queue->GetQ1());

  QuotientMetricShortestPathPtr metric = static_pointer_cast<QuotientMetricShortestPath>(quotient_cover_queue->GetMetric());
  uint number_of_configurations_passed = metric->InterpolateAlongPath(q, path, q->GetRadius(), q_milestone);
  for(uint k = 0; k < number_of_configurations_passed; k++){
    path.erase(path.begin());
  }
  path.insert(path.begin(), q_milestone);
}
