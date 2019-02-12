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

        //############################################################################
        //Distance Functions
        //############################################################################
        virtual double DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to);

        double DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to);
        double DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to);

        double DistanceConfigurationConfigurationQ1(const Configuration *q_from, const Configuration *q_to);
        double DistanceNeighborhoodNeighborhoodQ1(const Configuration *q_from, const Configuration *q_to);

        double DistanceQ1(const Configuration *q_from, const Configuration *q_to);
        double DistanceX1(const Configuration *q_from, const Configuration *q_to);

        //############################################################################
        //Interpolate Functions
        //############################################################################
        virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp);
        virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration* q_interp);

        void Interpolate(const Configuration *q_from, Configuration *q_to);
        void InterpolateQ1(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp);


      // bool Interpolate(const Configuration*, Configuration*);
      // bool Interpolate(const Configuration* q_from, const Configuration* q_to, Configuration* q_output);
      // bool Interpolate(const Configuration*, const Configuration*, double step_size, Configuration*);
      // void InterpolateUntilNeighborhoodBoundary(const Configuration *q_center, const Configuration *q_desired, Configuration *q_out);
      protected:

        og::QuotientCover *quotient_cover;

    };
  }
}
