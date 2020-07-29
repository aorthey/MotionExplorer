#pragma once
#include <ompl/base/Constraint.h>
#include <ompl/util/RandomNumbers.h>

OMPL_CLASS_FORWARD(ConstraintIntersectionTransition);

class ConstraintIntersectionTransition : public ompl::base::ConstraintIntersection
{
public:

    ConstraintIntersectionTransition(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints);

    std::vector<ompl::base::ConstraintPtr> getConstraintsVec();

    void setRandomMode();
    int getMode();
    void setMode(int);

protected:
    std::vector<int> constraintMode;
    ompl::RNG randomNumberGenerator;
};
