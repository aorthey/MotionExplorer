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
        QuotientMetric() = delete;
        QuotientMetric(og::QuotientCover*); 

        virtual double DistanceConfigurationConfiguration(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to) = 0;

        double DistanceConfigurationNeighborhood(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to);
        double DistanceNeighborhoodNeighborhood(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to);

        double DistanceQ1(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to);
        double DistanceX1(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to);
      protected:

        og::QuotientCover *quotient_cover;

    };
  }
}
