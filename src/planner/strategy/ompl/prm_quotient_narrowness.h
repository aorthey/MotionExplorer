#pragma once
#include "prm_quotient.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class PRMQuotientNarrow: public og::PRMQuotient{
      public:
        PRMQuotientNarrow(const ob::SpaceInformationPtr &si, PRMQuotientNarrow *previous_);
        virtual ~PRMQuotientNarrow() override;
      protected:
        virtual PDF<Edge> GetEdgePDF() override;
    };
    class PRMQuotientNarrowEdgeDegree: public og::PRMQuotientNarrow{
      public:
        PRMQuotientNarrowEdgeDegree(const ob::SpaceInformationPtr &si, PRMQuotientNarrow *previous_);
      protected:
        virtual PDF<Edge> GetEdgePDF() override;
    };
    class PRMQuotientNarrowMinCut: public og::PRMQuotientNarrow{
      public:
        PRMQuotientNarrowMinCut(const ob::SpaceInformationPtr &si, PRMQuotientNarrow *previous_);
      protected:
        virtual PDF<Edge> GetEdgePDF() override;
    };

  };
};
