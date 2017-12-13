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
        PRMMultiSlice(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "");

        virtual ~PRMMultiSlice() override;

        void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        void setup() override;
        void clear() override;
        void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
        void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);


      protected:
        std::vector<base::PathPtr> solutions;

        std::vector<T*> slicespaces;
        bool foundKLevelSolution{false};

        std::vector<ob::SpaceInformationPtr> si_vec;
        std::vector<ob::ProblemDefinitionPtr> pdef_vec;
    };
  };
};
#include "prm_multislice.ipp"
