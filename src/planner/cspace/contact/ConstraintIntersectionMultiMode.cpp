#include "planner/cspace/contact/ConstraintIntersectionMultiMode.h"
#include "planner/cspace/contact/ConstraintMultiMode.h"
#include "planner/cspace/contact/ConstraintContactFixed.h"
#include "planner/cspace/contact/ProjectedStateSpaceMultiMode.h"
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

ConstraintIntersectionMultiMode::ConstraintIntersectionMultiMode(
    const unsigned int ambientDim, 
    std::vector<ompl::base::ConstraintPtr> constraints)
  : ConstraintIntersection(ambientDim, constraints)
{
}

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
    // std::cout << "Random mode:" << constraintMode_ << std::endl;
}

void ConstraintIntersectionMultiMode::setTransitionMode(
    const ConstraintMode &modeFrom, const ConstraintMode &modeTo)
{
    if(modeTo.isLargerAs(modeFrom))
    {
        setMode(modeTo);
        //This is a constraint removal, e.g. where we remove a contact from a
        //surface
        //We use active constraints from modeTo,
        //but update them with contact points from modeFrom (i.e. we need to
        //project onto the right contact point)
        for(uint i = 0; i < modeTo.getFixedConstraintIndices().size(); i++)
        {
          int idxTo = modeTo.getFixedConstraintIndices().at(i);
          for(uint j = 0; j < modeFrom.getFixedConstraintIndices().size(); j++)
          {
              int idxFrom = modeFrom.getFixedConstraintIndices().at(j);
              if(idxFrom == idxTo)
              {
                  const Math3D::Vector3& cp = modeFrom.getFixedConstraintContactPoints().at(j);
                  ompl::base::ConstraintPtr constraint = constraints_.at(idxTo);
                  auto constraintContactFixed = 
                    std::static_pointer_cast<ConstraintContactFixed>(constraint);
                  constraintContactFixed->setFixedContactPoint(cp);
                  break;
              }
          }
        }

    }else
    {
      //Adding a contact (or staying on a contact). 
      setMode(modeFrom);
    }
}

void ConstraintIntersectionMultiMode::engraveConstraintMode(ompl::base::State* state)
{
    constraintMode_.clearFixedConstraint();
    for(uint i = 0; i < constraints_.size(); i++)
    {
        ompl::base::ConstraintPtr constraint = constraints_.at(i);
        auto constraintContactFixed = std::dynamic_pointer_cast<ConstraintContactFixed>(constraint);
        if(constraintContactFixed != nullptr)
        {
          if(constraintContactFixed->getMode() > 0)
          {
            Vector3 x = 
              constraintContactFixed->getPos(*state->as<ompl::base::ConstrainedStateSpace::StateType>());

            constraintContactFixed->setFixedContactPoint(x);
            constraintMode_.addFixedConstraint(i, x);
          }
        }
    }

    // std::cout << "Fixed Constraints:" << constraintMode_.numberFixedConstraints() << std::endl;

    state->as<ompl::base::ProjectedStateSpaceMultiMode::StateType>()->setMode(constraintMode_);
}

void ConstraintIntersectionMultiMode::setMode(ConstraintMode mode)
{
    constraintMode_ = mode;
    for(uint i = 0; i < constraints_.size(); i++)
    {
        ompl::base::ConstraintPtr constraint = constraints_.at(i);
        // check if constraints in vector are transitionConstraints
        auto constraintMultiMode = std::static_pointer_cast<ConstraintMultiMode>(constraint);
        constraintMultiMode->setMode(constraintMode_.at(i));
    }
    for(uint i = 0; i < constraintMode_.getFixedConstraintIndices().size(); i++)
    {
      int idx = constraintMode_.getFixedConstraintIndices().at(i);
      const Math3D::Vector3& cp = constraintMode_.getFixedConstraintContactPoints().at(i);
      ompl::base::ConstraintPtr constraint = constraints_.at(idx);
      auto constraintContactFixed = std::static_pointer_cast<ConstraintContactFixed>(constraint);
      constraintContactFixed->setFixedContactPoint(cp);
    }
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
