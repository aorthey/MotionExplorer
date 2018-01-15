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

        virtual bool SampleGraph(ob::State *workState) override;

    };

  };
};
