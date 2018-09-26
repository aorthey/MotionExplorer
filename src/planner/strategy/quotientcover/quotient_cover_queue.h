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

    class QuotientChartCoverQueue: public og::QuotientChartCover
    {
      typedef og::QuotientChartCover BaseT;
    public:

      uint verbose{0};

      QuotientChartCoverQueue(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientChartCoverQueue(void);
      virtual void Grow(double t) override;
      void clear() override;
      virtual std::vector<Configuration*> ExpandNeighborhood(Configuration*, const int) = 0;

      virtual Configuration* SampleCoverBoundary() override;
      virtual void AddConfigurationToPDF(Configuration *q) override;
    private:
      bool firstRun{true};
      uint NUMBER_OF_EXPANSION_SAMPLES{0};
      const double goalBias{0.3};

      struct CmpConfigurationPtrs
      {
        bool operator()(const Configuration* lhs, const Configuration* rhs) const
        {
           return lhs->GetImportance() < rhs->GetImportance();
        }
      };
      std::priority_queue<Configuration*, std::vector<Configuration*>, CmpConfigurationPtrs> priority_configurations;
    };
  }
}

