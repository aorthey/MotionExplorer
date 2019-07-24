#pragma once
#include "expansion.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class CoverExpansionStrategyRandomVoronoi: public CoverExpansionStrategy{

      public:

        CoverExpansionStrategyRandomVoronoi() = delete;
        CoverExpansionStrategyRandomVoronoi(og::QuotientCoverQueue*);

        virtual double Step() override;
    };
  }
}
