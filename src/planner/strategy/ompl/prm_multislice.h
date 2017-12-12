#pragma once
#include "prm_slice.h"
#include <type_traits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    template <class T>
    class PRMMultiSlice: public ob::Planner{
        static_assert(std::is_base_of<og::PRMSlice, T>::value, "Template must inherit from PRMSlice");

      public:
        PRMMultiSlice(std::vector<ob::SpaceInformationPtr> &si_vec);

        ~PRMMultiSlice() override;

        void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        void setup() override;
        void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef);

      protected:
        std::vector<base::PathPtr> solutions;

        std::vector<T*> slicespaces;
        bool foundKLevelSolution{false};
    };
  };
};
#include "prm_multislice.ipp"
