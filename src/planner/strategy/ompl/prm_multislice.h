#pragma once
#include "prm_slice.h"

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

        void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        //void clear() override;
        void setup() override;

        void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef);


      protected:
        std::vector<PRMSlice*> slicespaces;
        bool foundKLevelSolution{false};
    };
  };
};
