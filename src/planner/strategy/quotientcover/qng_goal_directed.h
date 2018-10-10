#pragma once
#include "planner/strategy/quotientcover/qng.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    class QNGGoalDirected: public og::QNG
    {
      typedef og::QNG BaseT;
      int verbose{0};
    public:

      QNGGoalDirected(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNGGoalDirected(void);

    };
  }
}


