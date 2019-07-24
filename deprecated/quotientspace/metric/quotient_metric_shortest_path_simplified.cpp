#include "quotient_metric_shortest_path_simplified.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

QuotientMetricShortestPathSimplified::QuotientMetricShortestPathSimplified(og::QuotientCover* quotient_cover_):
  QuotientMetricShortestPath(quotient_cover_)
{
}

//############################################################################
//Distance Functions
//############################################################################
double QuotientMetricShortestPathSimplified::DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to) 
{
  return DistanceQ1(q_from, q_to);
}
