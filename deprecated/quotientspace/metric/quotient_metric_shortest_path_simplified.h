#pragma once
#include "planner/strategy/quotient/metric/quotient_metric_shortest_path.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientMetricShortestPathSimplified: public QuotientMetricShortestPath
    {
        typedef og::QuotientMetric BaseT;
      public:
        typedef og::QuotientCover::Configuration Configuration;

        QuotientMetricShortestPathSimplified() = delete;
        QuotientMetricShortestPathSimplified(og::QuotientCover*); 

        //############################################################################
        //Distance Functions
        //############################################################################
        virtual double DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to) override;


    };
  }
}
