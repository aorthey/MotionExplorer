#pragma once
#include "expansion.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class CoverExpansionStrategyGoal: public CoverExpansionStrategy{

      typedef CoverExpansionStrategy BaseT;
      public:

        CoverExpansionStrategyGoal() = delete;
        CoverExpansionStrategyGoal(og::QuotientCoverQueue*);

        virtual double Step() override;
        virtual void Clear() override;
      private:
        Configuration *q_target{nullptr};
    };
  }
}
