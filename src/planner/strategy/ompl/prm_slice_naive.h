#pragma once
#include "prm_slice.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class PRMSliceNaive: public og::PRMSlice{

      public:
        PRMSliceNaive(const ob::SpaceInformationPtr &si, PRMSliceNaive *previous_);

        ~PRMSliceNaive() override;

        virtual bool Sample(ob::State *workState) override;
        bool SampleGraph(ob::State *workState);

      protected:

        ob::SpaceInformationPtr si_current;    //full configuration space Mi
        ob::SpaceInformationPtr si_standalone; //configuration space Ci = Mi/Mi-1

        base::StateSamplerPtr C1_sampler;

        PRMSliceNaive *previous;
    };

  };
};
