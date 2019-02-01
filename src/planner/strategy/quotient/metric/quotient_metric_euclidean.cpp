#include "quotient_metric_euclidean.h"
using namespace ompl::geometric;
using Configuration = QuotientCover::Configuration;

QuotientMetricEuclidean::QuotientMetricEuclidean(og::QuotientCover* quotient_cover_):
  QuotientMetric(quotient_cover_)
{
}

double QuotientMetricEuclidean::DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to) 
{
  return DistanceQ1(q_from, q_to);
}

void QuotientMetricEuclidean::Interpolate(const Configuration *q_from, const Configuration *q_to, double step_size, Configuration *q_interp)
{
  return quotient_cover->GetQ1()->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
}
