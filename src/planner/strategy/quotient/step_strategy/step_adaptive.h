#pragma once
#include "step.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class StepStrategyAdaptive: public og::StepStrategy{

        typedef og::StepStrategy BaseT;
        const int verbose{0};
      public:

        StepStrategyAdaptive() = default;
        StepStrategyAdaptive(og::QuotientCoverQueue*);

        virtual bool Towards(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to) override;
        virtual bool ExpandOutside(og::QuotientCover::Configuration *q_from) override;
        virtual bool ExpandRandom(og::QuotientCover::Configuration *q_from) override;
        virtual bool ExpandVoronoi() override;

      private:
        bool ChooseBestDirection(const std::vector<og::QuotientCover::Configuration*> &q_children, bool addBestToPriorityQueue = false);
        bool ChooseBestDirectionOnlyAddBestToQueue(const std::vector<og::QuotientCover::Configuration*> &q_children);
        bool ExpandTowardsSteepestAscentDirectionFromInitialDirection(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_initial);

        uint GetLargestNeighborhoodIndex(const std::vector<og::QuotientCover::Configuration*> &q_children);
        uint GetLargestNeighborhoodIndexOutsideCover(const std::vector<QuotientCover::Configuration*> &q_children);

        bool ConfigurationHasNeighborhoodLargerThan(og::QuotientCover::Configuration *q, double radius);

        std::vector<og::QuotientCover::Configuration*> GenerateCandidateDirections(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to);
    };
  }
}

