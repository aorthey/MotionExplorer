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

    //Quotient-space sufficient Neighborhood Graph planner (QNG)
    class QNG: public og::QuotientChartCover
    {
      typedef og::QuotientChartCover BaseT;
    public:

      uint verbose{0};

      QNG(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNG(void);
      virtual void Grow(double t) override;
      virtual void setup() override;
      virtual void clear() override;
      Configuration* EstimateBestNextState(Configuration *q_last, Configuration *q_current);

      virtual Configuration* SampleCoverBoundary() override;
      virtual void AddConfigurationToPDF(Configuration *q) override;
      virtual Configuration* SampleQuotientCover(ob::State *state) override;

      void AddToFastTrackConditional(std::vector<Configuration*>);
      std::vector<Configuration*> ExpandNeighborhood(Configuration*);
      void ExpandSubsetNeighborhood(const Configuration*, const Configuration*, std::vector<Configuration*>&);

      const double shortestPathBias{1.0};
      const double importanceSamplingBias{0.0};
      uint NUMBER_OF_EXPANSION_SAMPLES{0};

    private:

      bool firstRun{true};

      struct CmpConfigurationPtrs
      {
        bool operator()(const Configuration* lhs, const Configuration* rhs) const
        {
          //most important element to the top
           return lhs->GetImportance() < rhs->GetImportance();
        }
      };
      std::priority_queue<Configuration*, std::vector<Configuration*>, CmpConfigurationPtrs> priority_configurations;
    };
  }
}

