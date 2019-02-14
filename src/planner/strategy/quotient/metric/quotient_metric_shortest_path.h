#pragma once
#include "planner/strategy/quotient/metric/quotient_metric.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientMetricShortestPath: public QuotientMetric
    {
        typedef og::QuotientMetric BaseT;
      public:
        typedef og::QuotientCover::Configuration Configuration;

        QuotientMetricShortestPath() = delete;
        QuotientMetricShortestPath(og::QuotientCover*); 

        //############################################################################
        //Distance Functions
        //############################################################################
        virtual double DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to);

        //############################################################################
        //Interpolate Functions
        //############################################################################
        virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp) override;
        virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration *q_interp) override;

        std::vector<const Configuration*> GetInterpolationPath(const Configuration *q_from, const Configuration *q_to);
        uint InterpolateAlongPath(const Configuration *q_from, std::vector<const Configuration*> path, const double step_size, Configuration *q_interp);


    };
  }
}
