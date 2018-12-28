#pragma once
#include "planner/strategy/quotient/quotient_cover_queue.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class StepStrategy{

      public:
        StepStrategy();
        //Towards: Start at configuration q_from, and move a step into the
        //direction of q_to. This step can be made straight, or it can deviate
        //because of adjacent obstacles
        //return bool: if we successfully stepped or not
        virtual bool Towards(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to) = 0;
        //Expand: Start at q_from and move into the direction away from the
        //parent. This step can be done straight, or it can be perturbed by
        //adjacent obstacles
        //return bool: if we successfully stepped or not
        virtual bool Expand(og::QuotientCover::Configuration *q_from) = 0;

        void SetSpace(og::QuotientCoverQueue*);


      protected:
        og::QuotientCoverQueue *quotient_cover_queue;
    };
  }
}
