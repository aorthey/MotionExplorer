#pragma once
#include "planner/strategy/quotientcover/quotient_cover_queue.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //Quotient-space sufficient Neighborhood Graph planner (QNG)
    class QNG: public og::QuotientChartCoverQueue
    {
      typedef og::QuotientChartCoverQueue BaseT;
    public:

      uint verbose{0};

      QNG(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNG(void);

      //virtual Configuration* SampleCoverBoundary() override;
      //virtual void AddConfigurationToPDF(Configuration *q) override;
      //virtual Configuration* SampleUniformQuotientCover(ob::State *state) override;

      virtual std::vector<Configuration*> ExpandNeighborhood(Configuration*, const int) override;
      //void ExpandSubsetNeighborhood(const Configuration*, const Configuration*, std::vector<Configuration*>&, const int M_samples);

      //const double shortestPathBias{1.0};
      //const double importanceSamplingBias{0.0};

    };
  }
}

