#pragma once
#include "planner/strategy/quotient/quotient_cover.h"
#include "planner/strategy/quotient/metric/quotient_metric.h"


namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientMetricEuclidean: public QuotientMetric
    {
      public:
        QuotientMetricEuclidean() = delete;
        QuotientMetricEuclidean(og::QuotientCover*); 

        virtual double DistanceConfigurationConfiguration(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to) override;
        virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, double step, Configuration* q_interp) override;

    };
  }
}
