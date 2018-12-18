#pragma once
#include "step.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class StepStrategyAdaptive: public og::StepStrategy{

      public:

        StepStrategyAdaptive() = default;

        virtual bool Towards(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to) override;

      private:
        uint GetLargestNeighborhoodIndex(const std::vector<og::QuotientCover::Configuration*> &q_children);

        bool ConfigurationHasNeighborhoodLargerThan(og::QuotientCover::Configuration *q, double radius);

        std::vector<og::QuotientCover::Configuration*> GenerateCandidateDirections(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to);
    };
  }
}

