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
        //return bool: if we successfully stepped or not
        virtual bool Towards(og::QuotientCover::Configuration *q_from, og::QuotientCover::Configuration *q_to) = 0;

        void SetSpace(og::QuotientCoverQueue*);


      protected:
        og::QuotientCoverQueue *quotient_cover_queue;
    };
  }
}
