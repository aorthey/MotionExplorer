#pragma once
#include "step.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class StepStrategyStraight: public og::StepStrategy{

      public:

        StepStrategyStraight() = delete;
        StepStrategyStraight(og::QuotientCoverQueue*);

        virtual bool Towards(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to) override;

      private:

        bool ConfigurationHasNeighborhoodLargerThan(og::QuotientCover::Configuration *q, double radius);

        std::vector<og::QuotientCover::Configuration*> GenerateCandidateDirections(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to);
    };
  }
}
