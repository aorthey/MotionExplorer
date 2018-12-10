#pragma once
#include "quotient_cover.h"
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

    class QuotientCoverQueue: public og::QuotientCover
    {
      typedef og::QuotientCover BaseT;
    public:

      uint verbose{0};

      QuotientCoverQueue(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientCoverQueue(void);
      virtual void Grow(double t) override;
      void clear() override;
      void setup() override;
      virtual std::vector<Configuration*> ExpandNeighborhood(Configuration*, const int) = 0;

      virtual Configuration* SampleCoverBoundary() override;
      virtual void AddConfigurationToPDF(Configuration *q) override;
      virtual void Print(std::ostream& out) const override;
      virtual Configuration* SampleUniformQuotientCover(ob::State *state) override;

    protected:
      bool firstRun{true};
      uint NUMBER_OF_EXPANSION_SAMPLES{0};
      const double goalBias{0.5};
      const double shortestPathBias{1.0};

      struct CmpConfigurationPtrs
      {
        // ">" operator: smallest value is top in queue
        // "<" operator: largest value is top in queue (default)
        bool operator()(const Configuration* lhs, const Configuration* rhs) const
        {
           return lhs->GetImportance() < rhs->GetImportance();
        }
      };
      typedef std::priority_queue<Configuration*, std::vector<Configuration*>, CmpConfigurationPtrs> ConfigurationPriorityQueue;
      ConfigurationPriorityQueue priority_configurations;

    public:
      const ConfigurationPriorityQueue& GetPriorityQueue();
    };
  }
}

