#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_DRRTIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_DRRTIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
        OMPL_CLASS_FORWARD(StateValidityCheckerDifferentiable);
        OMPL_CLASS_FORWARD(GoalStateDifferentiable);
    }
    namespace multilevel
    {
        /** \brief Implementation of BundleSpace Rapidly-Exploring Random Trees Algorithm*/
        class DRRTImpl : public ompl::multilevel::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;

        public:
            DRRTImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~DRRTImpl() override;

            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;
            virtual void setup() override;

        protected:
            Configuration* steerAlongGradient(Configuration *xcurrent);
            Configuration* leapfrogAlongPotential(Configuration *x);
            bool steer(const base::State *from, 
                const Eigen::VectorXd& gradient, base::State *to);
            bool leapfrog(base::State *from, int steps);
            Eigen::VectorXd getGradient(const base::State *state);

            const base::StateValidityCheckerDifferentiable* differentiableConstraint_{nullptr};
            base::GoalStateDifferentiablePtr differentiableGoal_{nullptr};

            bool hasDifferentiableConstraints_{false};
            bool hasDifferentiableGoal_{false};

            base::State *tmp_;

        };
    }  // namespace multilevel
}  // namespace ompl

#endif

