#include "planner/cspace/contact/ConstraintIntersection_Transition.h"
#include <ompl/base/Constraint.h>

ConstraintIntersectionTransition::ConstraintIntersectionTransition(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints)
: ConstraintIntersection(ambientDim, constraints){}

std::vector<ompl::base::ConstraintPtr> ConstraintIntersectionTransition::getConstraintsVec() 
{
    return constraints_;
}
