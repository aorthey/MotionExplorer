#pragma once
#include "planner/strategy/quotient/quotient_cover.h"

//namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientCoverQueue;
    typedef QuotientCover::Configuration Configuration;

    class CoverExpansionStrategy{

      const uint verbose{1};
      public:
        CoverExpansionStrategy() = delete;
        CoverExpansionStrategy(QuotientCoverQueue*);
        virtual void Clear();

        //Step
        //Returns:
        //  double d \in {-1} \cup [0,\infty[
        //ratio of radius of new neighborhood compared
        //to old neighborhood. I.e. if neighborhoods are not changing in size,
        //then return value = 1. If neighborhood could not be extended, we
        //return -1. 
        //-- Could be used for multi-arm bandit algorithm
        virtual double Step() = 0;

      protected:
        QuotientCoverQueue *quotient_cover_queue;
        Configuration *q_last_expanded{nullptr};

        //Towards: Start at configuration q_from, and move a step into the
        //direction of q_to. This step can be made straight, or it can deviate
        //because of adjacent obstacles
        //return bool: if we successfully stepped or not
        bool TowardsStraightLine(Configuration *q_from, Configuration *q_to);

        //Move from q_from to another point q_to in the configuration space
        bool Towards(QuotientCover::Configuration *q_from, QuotientCover::Configuration *q_to);
        //Move from q_from to another point q_to which lies ON the boundary of
        //the neighborhood of q_from
        bool TowardsBoundaryPoint(Configuration*, Configuration*);

        bool ExpandTowardsSteepestAscentDirectionFromInitialDirection(const Configuration *q_from, Configuration *q_initial);
    };
  }
}
