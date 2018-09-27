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
      int verbose{0};
    public:

      QNG(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNG(void);

      virtual std::vector<Configuration*> ExpandNeighborhood(Configuration*, const int) override;

    };
  }
}

