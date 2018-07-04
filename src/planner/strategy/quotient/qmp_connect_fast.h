#pragma once
#include "qmp_connect.h"
#include <CGAL/Triangulation_data_structure.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QMPConnectFast: public og::QMPConnect{

        typedef og::QMPConnect BaseT;
        typedef CGAL::Triangulation_data_structure<CGAL::Dynamic_dimension_tag> TDS;
      public:

        QMPConnectFast(const ob::SpaceInformationPtr &si, Quotient *previous_);
        ~QMPConnectFast() override;
        bool Sample(ob::State *q_random) override;
        void Grow(double t) override;
      protected:
        Graph G_infeasible;
        Graph G_feasible;
        TDS *simplex;
    };

  };
};

