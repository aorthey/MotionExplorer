#pragma once
#include "planner/strategy/quotient/quotient.h"
#include "planner/strategy/quotientchart/quotient_chart.h"
#include <type_traits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    template <class T>
      //MultiChart: Works like MultiQuotient, but whenever a quotientchart finds
      //a new path, the space is split such that we have a new quotientchart
      //having a graph based on the new found solution plus surrounding
      //vertices.
      //
    class MultiChart: public ob::Planner{
        static_assert(std::is_base_of<og::QuotientChart, T>::value, "Template must inherit from QuotientChart");

      public:
        MultiChart(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "QCP");

        virtual ~MultiChart() override;

        void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        void setup() override;
        void clear() override;
        void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
        void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);

      protected:
        std::vector<base::PathPtr> solutions;

        og::QuotientChart* root{nullptr};
        std::vector<og::QuotientChart*> quotientSpaces; //only used to project lower level ob::states into the configuration space (TODO: make this more efficient)

        std::vector<uint> current_node;

        uint levels{0};
        bool found_path_on_last_level{false};
        bool saturated_levels{false};

        std::vector<ob::SpaceInformationPtr> si_vec;
        std::vector<ob::ProblemDefinitionPtr> pdef_vec;
    };
  };
};
#include "multichart.ipp"
