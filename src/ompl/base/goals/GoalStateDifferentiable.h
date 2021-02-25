#ifndef OMPL_BASE_GOALS_GOAL_STATE_DIFFERENTIABLE_
#define OMPL_BASE_GOALS_GOAL_STATE_DIFFERENTIABLE_

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/ScopedState.h"

namespace ompl
{
    namespace base
    {
        class GoalStateDifferentiable : public GoalStates
        {
        public:
            GoalStateDifferentiable(const SpaceInformationPtr &si);

            virtual ~GoalStateDifferentiable() override;

            virtual double cost(const State *state) const;

            virtual Eigen::VectorXd costGradient(const State *state) const;
        protected:
            base::State *neighbor_;
        };
    }
}

#endif



