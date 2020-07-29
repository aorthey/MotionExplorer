#pragma once
#include "planner/cspace/contact/ConstraintMode.h"
#include <ompl/base/Constraint.h>

OMPL_CLASS_FORWARD(ConstraintIntersectionTransition);

class ConstraintIntersectionMultiMode : public ompl::base::ConstraintIntersection
{
public:

    ConstraintIntersectionMultiMode(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints);

    std::vector<ompl::base::ConstraintPtr> getConstraintsVec();

    void setRandomMode();

    void setInitMode();

    void setGoalMode();

    ConstraintMode getMode();

    void setMode(ConstraintMode);

protected:
    ConstraintMode constraintMode_;
};
using ConstraintIntersectionMultiModePtr = std::shared_ptr<ConstraintIntersectionMultiMode>;

