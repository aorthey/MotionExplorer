#pragma once
#include "prm_slice_naive.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class PRMMultiSlice: public ob::Planner{

      public:
        PRMMultiSlice(std::vector<ob::SpaceInformationPtr> &si_vec);

        ~PRMMultiSlice() override;

        //void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        //void clear() override;
        void setup() override;

      protected:
        std::vector<PRMSliceNaive*> slicespaces;
        bool foundKLevelSolution{false};
    };
  };
};
