#pragma once
#include "quotient_chart_complex.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/datastructures/PDF.h>
#include <boost/graph/random.hpp> 
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
  namespace geometric
  {
    class QSP: public og::QuotientChart{
      typedef og::QuotientChart BaseT;
      public:

        QSP(const ob::SpaceInformationPtr &si, QuotientChart *parent_ = nullptr);
        virtual ~QSP() override;
        void Grow(double t) override;
        bool Sample(ob::State *q_random) override;
        void getPlannerData(ob::PlannerData &data) const;
        void setup() override;

      private:
        bool checkSufficiency{false};
        OMPLValidityCheckerNecessarySufficientPtr checker;
        uint number_of_samples;
        uint number_of_infeasible_samples;
        uint number_of_sufficient_samples;

    };

  };
};
