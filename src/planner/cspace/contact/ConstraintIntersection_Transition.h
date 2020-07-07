#pragma once
#include <ompl/base/Constraint.h>

OMPL_CLASS_FORWARD(ConstraintIntersectionTransition);

class ConstraintIntersectionTransition : public ompl::base::ConstraintIntersection
{
public:

    ConstraintIntersectionTransition(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints);

    std::vector<ompl::base::ConstraintPtr> getConstraintsVec();
};
