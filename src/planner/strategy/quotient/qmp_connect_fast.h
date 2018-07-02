#pragma once
#include "qmp_connect.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QMPConnectFast: public og::QMPConnect{

        typedef og::QMPConnect BaseT;
      public:

        QMPConnectFast(const ob::SpaceInformationPtr &si, Quotient *previous_);
        ~QMPConnectFast() override;
        bool Sample(ob::State *q_random) override;
        void Grow(double t) override;
    };

  };
};

