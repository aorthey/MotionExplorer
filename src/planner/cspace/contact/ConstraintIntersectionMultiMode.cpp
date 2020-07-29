#include "planner/cspace/contact/ConstraintIntersectionMultiMode.h"
#include "planner/cspace/contact/ConstraintMultiMode.h"
#include <ompl/base/Constraint.h>

ConstraintIntersectionMultiMode::ConstraintIntersectionMultiMode(const unsigned int ambientDim, std::vector<ompl::base::ConstraintPtr> constraints)
: ConstraintIntersection(ambientDim, constraints){}

std::vector<ompl::base::ConstraintPtr> ConstraintIntersectionMultiMode::getConstraintsVec() 
{
    return constraints_;
}

ConstraintMode ConstraintIntersectionMultiMode::getMode()
{
    return constraintMode_;
}

void ConstraintIntersectionMultiMode::setRandomMode()
{
    constraintMode_.clear();
    while(!constraintMode_.isValid())
    {
      constraintMode_.clear();
      for(uint i = 0; i < constraints_.size(); i++)
      {
          ompl::base::ConstraintPtr constraint = constraints_.at(i);
          // check if constraints in vector are transitionConstraints
          auto constraintMultiMode = std::dynamic_pointer_cast<ConstraintMultiMode>(constraint);
          if (constraintMultiMode != nullptr)
          {
              constraintMultiMode->setRandomMode();
              constraintMode_.add(constraintMultiMode->getMode());
          }
      }
    }
}

void ConstraintIntersectionMultiMode::setInitMode()
{
    constraintMode_.clear();
    for(uint i = 0; i < constraints_.size(); i++)
    {
        ompl::base::ConstraintPtr constraint = constraints_.at(i);
        // check if constraints in vector are transitionConstraints
        auto constraintMultiMode = std::dynamic_pointer_cast<ConstraintMultiMode>(constraint);
        if (constraintMultiMode != nullptr)
        {
            constraintMultiMode->setMode(1);
            constraintMode_.add(1);
        }
    }
}

void ConstraintIntersectionMultiMode::setMode(ConstraintMode mode)
{
  constraintMode_ = mode;
}

void ConstraintIntersectionMultiMode::setGoalMode()
{
    constraintMode_.clear();
    for(uint i = 0; i < constraints_.size(); i++)
    {
        ompl::base::ConstraintPtr constraint = constraints_.at(i);
        // check if constraints in vector are transitionConstraints
        auto constraintMultiMode = std::dynamic_pointer_cast<ConstraintMultiMode>(constraint);
        if (constraintMultiMode != nullptr)
        {
            int r = constraintMultiMode->getNumberOfModes() - 1;
            constraintMultiMode->setMode(r);
            constraintMode_.add(r);
        }
    }
}
