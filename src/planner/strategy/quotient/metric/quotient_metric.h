#pragma once
#include "planner/strategy/quotient/quotient_cover.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientMetric
    {
      public:
        typedef og::QuotientCover::Configuration Configuration;

        QuotientMetric() = delete;
        QuotientMetric(og::QuotientCover*); 

        virtual double DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to) = 0;

        double DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to);
        double DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to);

        double DistanceQ1(const Configuration *q_from, const Configuration *q_to);
        double DistanceX1(const Configuration *q_from, const Configuration *q_to);

        void InterpolateQ1(const Configuration *q_from, const Configuration *q_to, double step, Configuration* q_out);
      protected:

        og::QuotientCover *quotient_cover;

    };
  }
}
