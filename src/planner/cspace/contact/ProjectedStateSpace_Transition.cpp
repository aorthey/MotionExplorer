#include "planner/cspace/contact/ProjectedStateSpace_Transition.h"
#include "planner/cspace/contact/ConstraintIntersection_Transition.h"
#include "planner/cspace/contact/TransitionConstraint2D.h"
#include "planner/cspace/contact/TransitionConstraint3D.h"
#include <utility>

/// ProjectedStateSampler
/// Public

ompl::base::ProjectedStateSamplerTransition::ProjectedStateSamplerTransition(const ProjectedStateSpaceTransition *space, StateSamplerPtr sampler)
  : ProjectedStateSampler(reinterpret_cast<const ProjectedStateSpace *>(space), std::move(sampler))
  , constraint_(space->getConstraint()) 
{
    // check if multiple (constraintIntersection) or single constraint
    constraintIntersection_ =
      std::dynamic_pointer_cast<ConstraintIntersectionTransition>(space->getConstraint());
    if (constraintIntersection_ != nullptr) 
    {
        constraintsVec = constraintIntersection_->getConstraintsVec();
    }
}

void ompl::base::ProjectedStateSamplerTransition::sampleUniform(State *state)
{
    WrapperStateSampler::sampleUniform(state);

    constraintIntersection_->setRandomMode();

    state->as<StateType>()->setMode(constraintIntersection_->getMode());
    
    constraint_->project(state);

    space_->enforceBounds(state);

    for(uint i = 0; i < constraintsVec.size(); i++)
    {
        ConstraintPtr cPi = constraintsVec.at(i);
        auto tCP = std::dynamic_pointer_cast<TransitionConstraint2D>(cPi);
        auto tCP3D = std::dynamic_pointer_cast<TransitionConstraint3D>(cPi);
        if (tCP != nullptr){
            tCP->setMode(NO_ACTIVE_CONSTRAINT);

        } else if (tCP3D != nullptr){
            tCP3D->setMode(NO_ACTIVE_CONSTRAINT);
        }
    }
}
void ompl::base::ProjectedStateSamplerTransition::sampleUniformNear(
    State *state, const State *near, double distance)
{
  OMPL_ERROR("NYI");
  ompl::Exception("NYI");
}
void ompl::base::ProjectedStateSamplerTransition::sampleGaussian(
    State *state, const State *mean, double stdDev)
{
  OMPL_ERROR("NYI");
  ompl::Exception("NYI");
}

bool ompl::base::ProjectedStateSpaceTransition::discreteGeodesic(const State *from, const State *to, bool interpolate,
                                                       std::vector<State *> *geodesic) const
{
    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

    //#########################################################################
    //Set Transition constraints to be inactive
    ob::ConstraintPtr constraint = getConstraint();
    auto constraintIntersection = 
      std::dynamic_pointer_cast<ConstraintIntersectionTransition>(getConstraint());
    if (constraintIntersection != nullptr) {
        std::vector<ob::ConstraintPtr> constraintsVec = 
            constraintIntersection->getConstraintsVec();
        for(uint i = 0; i < constraintsVec.size(); i++)
        {
            ConstraintPtr cPi = constraintsVec.at(i);
            auto tCP = std::dynamic_pointer_cast<TransitionConstraint2D>(cPi);
            auto tCP3D = std::dynamic_pointer_cast<TransitionConstraint3D>(cPi);
            if (tCP != nullptr){
                tCP->setMode(NO_ACTIVE_CONSTRAINT);

            } else if (tCP3D != nullptr){
                tCP3D->setMode(NO_ACTIVE_CONSTRAINT);
            }
        }
    }
    //#########################################################################

    const double tolerance = delta_;

    // No need to traverse the manifold if we are already there.
    double dist, step, total = 0;
    if ((dist = distance(from, to)) <= tolerance)
        return true;

    const double max = dist * lambda_;

    auto previous = cloneState(from);
    auto scratch = allocState();

    auto &&svc = si_->getStateValidityChecker();

    do
    {
        WrapperStateSpace::interpolate(previous, to, delta_ / dist, scratch);

        // Project new state onto constraint manifold
        if (!constraint_->project(scratch)                  // not on manifold
            || !(interpolate || svc->isValid(scratch))      // not valid
            || (step = distance(previous, scratch)) > lambda_ * delta_)  // deviated
            break;

        // Check if we have wandered too far
        total += step;
        if (total > max)
            break;

        // Check if we are no closer than before
        const double newDist = distance(scratch, to);
        if (newDist >= dist)
            break;

        dist = newDist;
        copyState(previous, scratch);

        // Store the new state
        if (geodesic != nullptr)
            geodesic->push_back(cloneState(scratch));

    } while (dist >= tolerance);

    freeState(scratch);
    freeState(previous);

    return dist <= tolerance;
}
// unsigned int ompl::base::ProjectedStateSpaceTransition::validSegmentCount(const State *state1, const State *state2) const
// {
//   return 1;
// }

