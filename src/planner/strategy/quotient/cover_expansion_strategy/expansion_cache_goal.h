#pragma once
#include "expansion_cache.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientCoverQueue;

    //Like ExpansionGoal, but cached
    class CoverExpansionStrategyCacheGoal: public CoverExpansionStrategyCache
    {
      public:
        CoverExpansionStrategyCacheGoal() = delete;
        CoverExpansionStrategyCacheGoal(QuotientCoverQueue*);
        virtual double Step() override;
    };
  }
}
