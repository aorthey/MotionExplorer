#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorer_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorer_
#include "planner/strategy/quotient/quotient.h"
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        template <class T>
        class MotionExplorer : public ob::Planner
        {
            static_assert(std::is_base_of<og::Quotient, T>::value, "Template must inherit from Quotient");

        public:
            const bool DEBUG{false};

            MotionExplorer(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "MotionExplorer");

            void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);

            virtual ~MotionExplorer() override;

            void getPlannerData(ob::PlannerData &data) const override;

            //Write different PTC (for reaching topological phase shift,
            //etcetera)
            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

            void setup() override;
            void clear() override;
            void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;

        protected:
            //std::vector<ob::PathPtr> solutions_;

            std::vector<int> selection;
            bool topologicalPhaseShift;

            double pathBias{0.8}; //[0,1]

            /// Sequence of quotient-spaces
            std::vector<og::Quotient *> quotientSpaces_;
            std::vector<uint> currentPath;

            std::vector<ob::SpaceInformationPtr> siVec_;
            std::vector<ob::ProblemDefinitionPtr> pdefVec_;
        };
    }
}
#include "Explorer.ipp"
#endif


