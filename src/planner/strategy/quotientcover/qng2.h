#pragma once
#include "planner/strategy/quotientcover/quotient_cover.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    //Quotient-space sufficient Neighborhood Graph planner (QNG2)
    class QNG2: public og::QuotientChartCover
    {
      typedef og::QuotientChartCover BaseT;
    public:

      const uint verbose{1};
      bool firstRun{true};

      QNG2(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNG2(void);
      virtual void Grow(double t) override;
      virtual void setup() override;
      virtual void clear() override;

      virtual void AddConfigurationToPDF(Configuration *q) override;
      virtual Configuration* SampleQuotientCover(ob::State *state) override;
      virtual Configuration* Sample() override;
      void ConnectRecurseLargest(Configuration *q_from, Configuration *q_next);

      const double shortestPathBias{0.0};
      uint NUMBER_OF_EXPANSION_SAMPLES{0};
    };
  }
}

