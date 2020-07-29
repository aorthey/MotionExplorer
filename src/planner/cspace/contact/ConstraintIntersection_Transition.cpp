#include "planner/cspace/contact/ConstraintIntersection_Transition.h"
#include "planner/cspace/contact/TransitionConstraint2D.h"
#include "planner/cspace/contact/TransitionConstraint3D.h"
#include <ompl/base/Constraint.h>

ConstraintIntersectionTransition::ConstraintIntersectionTransition(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints)
: ConstraintIntersection(ambientDim, constraints){}

std::vector<ompl::base::ConstraintPtr> ConstraintIntersectionTransition::getConstraintsVec() 
{
    return constraints_;
}

void ConstraintIntersectionTransition::setRandomMode()
{
    for(uint i = 0; i < constraints_.size(); i++)
    {
        ompl::base::ConstraintPtr cPi = constraints_.at(i);
        // check if constraints in vector are transitionConstraints
        auto tCP = std::dynamic_pointer_cast<TransitionConstraint2D>(cPi);
        auto tCP3D = std::dynamic_pointer_cast<TransitionConstraint3D>(cPi);
        if (tCP != nullptr)
        {
            int newMode = randomNumberGenerator.uniformInt(0,2);

            tCP->setMode(newMode);

        } else if(tCP3D != nullptr)
        {
            int newMode = randomNumberGenerator.uniformInt(0,2);

            tCP3D->setMode(newMode);
        }

    }
}
