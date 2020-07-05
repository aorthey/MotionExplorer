#pragma once
#include <ompl/base/Constraint.h>


class ConstraintIntersectionTransition : public ompl::base::ConstraintIntersection
{
public:

    ConstraintIntersectionTransition(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints);

    ompl::base::ConstraintPtr getConstraintPtr();
};
