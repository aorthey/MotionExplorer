#pragma once
#include "step.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class StepStrategyStraight: public og::StepStrategy{

      public:

        StepStrategyStraight() = default;

        virtual bool Towards(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to) override;
        virtual bool Expand(og::QuotientCover::Configuration *q_from) override;

      private:

        bool ConfigurationHasNeighborhoodLargerThan(og::QuotientCover::Configuration *q, double radius);

        std::vector<og::QuotientCover::Configuration*> GenerateCandidateDirections(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to);
    };
  }
}

