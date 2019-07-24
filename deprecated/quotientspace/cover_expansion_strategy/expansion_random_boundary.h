#pragma once
#include "expansion.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class CoverExpansionStrategyRandomBoundary: public CoverExpansionStrategy{

      public:

        CoverExpansionStrategyRandomBoundary() = delete;
        CoverExpansionStrategyRandomBoundary(og::QuotientCoverQueue*);

        virtual double Step() override;
    };
  }
}
