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

        ob::SpaceInformationPtr M1;    //full configuration space Mi
        ob::SpaceInformationPtr C1; //configuration space Ci = Mi/Mi-1

        base::StateSamplerPtr C1_sampler;

        uint M0_subspaces;
        uint M1_subspaces;
        uint C1_subspaces;

        PRMSliceNaive *previous;
    };

  };
};
