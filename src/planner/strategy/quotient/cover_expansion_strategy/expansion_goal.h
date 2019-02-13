#pragma once
#include "expansion.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class CoverExpansionStrategyGoal: public CoverExpansionStrategy{

      public:

        CoverExpansionStrategyGoal() = delete;
        CoverExpansionStrategyGoal(og::QuotientCoverQueue*);

        virtual double Step() override;
    };
  }
}
