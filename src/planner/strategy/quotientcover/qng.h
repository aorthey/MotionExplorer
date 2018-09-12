#pragma once
#include "planner/strategy/quotientcover/quotient_cover.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>

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
    };
  }
}

