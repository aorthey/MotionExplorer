#pragma once
#include "prm_slice.h"

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
    class PRMSliceNarrow: public og::PRMSlice{
      public:
        PRMSliceNarrow(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_);
        virtual ~PRMSliceNarrow() override;
      protected:
        virtual PDF<Edge> GetEdgePDF() override;
    };
    class PRMSliceNarrowEdgeDegree: public og::PRMSliceNarrow{
      public:
        PRMSliceNarrowEdgeDegree(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_);
      protected:
        virtual PDF<Edge> GetEdgePDF() override;
    };
    class PRMSliceNarrowMinCut: public og::PRMSliceNarrow{
      public:
        PRMSliceNarrowMinCut(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_);
      protected:
        virtual PDF<Edge> GetEdgePDF() override;
    };

  };
};
