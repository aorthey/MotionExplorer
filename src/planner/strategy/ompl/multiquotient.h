#pragma once
#include "quotient.h"
#include <type_traits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    template <class T, typename Tlast=T>
    class MultiQuotient: public ob::Planner{
        static_assert(std::is_base_of<og::Quotient, T>::value, "Template must inherit from QuotientPlanner");

      public:
        MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "");

        virtual ~MultiQuotient() override;

        void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        void setup() override;
        void clear() override;
        void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
        void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);

      protected:
        std::vector<base::PathPtr> solutions;

        std::vector<og::Quotient*> quotientSpaces;
        bool foundKLevelSolution{false};

        std::vector<ob::SpaceInformationPtr> si_vec;
        std::vector<ob::ProblemDefinitionPtr> pdef_vec;
    };
  };
};
#include "multiquotient.ipp"
