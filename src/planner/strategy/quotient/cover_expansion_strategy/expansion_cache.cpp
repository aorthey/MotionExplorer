#include "expansion_cache.h"
#include "planner/strategy/quotient/quotient_cover_queue.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"
#include "planner/strategy/quotient/metric/quotient_metric_shortest_path.h"
#include <ompl/util/Time.h>

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
  std::cout << "CACHE PATH" << std::endl;
  ompl::time::point t_start = ompl::time::now();
  QuotientMetricShortestPathPtr metric = static_pointer_cast<QuotientMetricShortestPath>(quotient_cover_queue->GetMetric());
  cached_path = metric->GetInterpolationPath(q_from, q_to);
  double t_cache = ompl::time::seconds(ompl::time::now() - t_start);
  distance_next_milestone = metric->DistanceQ1(cached_path.at(0), cached_path.at(1));
  std::cout << "CACHED PATH contains " << cached_path.size() << " configs. (time: " << t_cache << ")" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

void CoverExpansionStrategyCache::RollUpPath()
{
  QuotientMetricShortestPathPtr metric = static_pointer_cast<QuotientMetricShortestPath>(quotient_cover_queue->GetMetric());
  ConfigurationPath &p = cached_path;
  if(verbose){
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "ROLL UP PATH: " << cached_path.size() << " configs." << std::endl;
    quotient_cover_queue->Print(p.at(0), false);
  }

  const Configuration *q1 = p.at(0);
  double radius = q1->GetRadius();
  if(radius < distance_next_milestone)
  {
    //Smooth Interpolation without crossing corners
    const Configuration *q2 = p.at(1);
    //double step = d_last_to_next - (d - q_from->GetRadius());
    metric->InterpolateQ1(q1, q2, radius/distance_next_milestone, q_milestone);

    distance_next_milestone -= radius;
    p.erase(p.begin());
    p.insert(p.begin(), q_milestone);
  }else{
    //Need to cut corners
    std::cout << "Interpolate with Cutting Corners" << std::endl;
    uint number_of_configurations_passed = metric->InterpolateAlongPath(q1, p, radius, q_milestone);
    for(uint k = 0; k < number_of_configurations_passed; k++){
      p.erase(p.begin());
    }
    p.insert(p.begin(), q_milestone);
    distance_next_milestone = metric->DistanceQ1(p.at(0),p.at(1));
  }

  if(verbose){
    std::cout << "Distance X0-X1: " << metric->DistanceQ1(p.at(0), p.at(1)) << std::endl;
    std::cout << "New Distance X0'-X1: " << metric->DistanceQ1(p.at(0), p.at(1)) << std::endl;
    std::cout << "PATH CHANGED TO " << p.size() << " configs." << std::endl;
    std::cout << std::string(80, '-') << std::endl;
  }
}
