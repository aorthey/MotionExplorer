#pragma once
#include "expansion.h"

namespace og = ompl::geometric;
namespace ompl
{
  namespace geometric
  {
    class QuotientCoverQueue;

    OMPL_CLASS_FORWARD(QuotientMetricShortestPath);
    typedef QuotientCover::Configuration Configuration;
    class CoverExpansionStrategyCache: public CoverExpansionStrategy
    {
      typedef CoverExpansionStrategy BaseT;
      public:
        CoverExpansionStrategyCache() = delete;
        CoverExpansionStrategyCache(QuotientCoverQueue*);
        virtual double Step() = 0;
        void Clear() override;

      protected:
        //compute shortest path once, then walk along it. This makes the
        //algorithm much more efficient, because otherwise we would recompute
        //this path for every single step
        void CachePath(const Configuration *q_from, const Configuration *q_to);

        typedef std::vector<const Configuration*> ConfigurationPath;
        ConfigurationPath cached_path;
        Configuration *q_source{nullptr};

        //Follow path for path.at(0)->GetRadius() to reach milestone. Then remove all
        //configurations before milestone on path. Finally add milestone as the
        //first element of path, and return. Path will be changed.
        void RollUpPath(ConfigurationPath &path);
        
     };
  }
}
