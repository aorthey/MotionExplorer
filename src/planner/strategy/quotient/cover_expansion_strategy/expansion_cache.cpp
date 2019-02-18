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
void CoverExpansionStrategyCache::Clear()
{
  q_source = nullptr;
  cached_path.clear();
  BaseT::Clear();
}
void CoverExpansionStrategyCache::CachePath(const Configuration *q_from, const Configuration *q_to)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "CACHE PATH" << std::endl;
  QuotientMetricShortestPathPtr metric = static_pointer_cast<QuotientMetricShortestPath>(quotient_cover_queue->GetMetric());
  cached_path = metric->GetInterpolationPath(q_from, q_to);
  std::cout << "CACHED PATH contains " << cached_path.size() << " configs." << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

void CoverExpansionStrategyCache::RollUpPath(ConfigurationPath &path)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "ROLL UP PATH: " << cached_path.size() << " configs." << std::endl;
  const Configuration *q = path.at(0);
  Configuration *q_milestone = new Configuration(quotient_cover_queue->GetQ1());

  QuotientMetricShortestPathPtr metric = static_pointer_cast<QuotientMetricShortestPath>(quotient_cover_queue->GetMetric());
  quotient_cover_queue->Print(q, false);
  std::cout << "Distance X0-X1: " << metric->DistanceQ1(path.at(0), path.at(1)) << std::endl;
  uint number_of_configurations_passed = metric->InterpolateAlongPath(path.at(0), path, path.at(0)->GetRadius(), q_milestone);
  for(uint k = 0; k < number_of_configurations_passed; k++){
    path.erase(path.begin());
  }
  path.insert(path.begin(), q_milestone);
  std::cout << "New Distance X0'-X1: " << metric->DistanceQ1(path.at(0), path.at(1)) << std::endl;
  std::cout << "passed " << number_of_configurations_passed << " configs during roll up of path." << std::endl;
  quotient_cover_queue->Print(q_milestone, false);
  std::cout << "PATH CHANGED TO " << cached_path.size() << " configs." << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  // static int k = 0;
  // k++;
  // if(k>2) exit(0);
}
