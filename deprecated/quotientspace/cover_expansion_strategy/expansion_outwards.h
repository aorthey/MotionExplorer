#pragma once
#include "expansion.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class CoverExpansionStrategyOutwards: public CoverExpansionStrategy{

      public:

        CoverExpansionStrategyOutwards() = delete;
        CoverExpansionStrategyOutwards(og::QuotientCoverQueue*);

        virtual double Step() override;
    };
  }
}
