#include "planner/cspace/contact/ProjectedStateSpace_Transition.h"
#include "planner/cspace/contact/ConstraintIntersection_Transition.h"
#include "planner/cspace/contact/TransitionConstraint.h"
#include <utility>

/// ProjectedStateSampler

/// Public

ompl::base::ProjectedStateSamplerTransition::ProjectedStateSamplerTransition(const ProjectedStateSpaceTransition *space, StateSamplerPtr sampler)
  : ProjectedStateSampler(reinterpret_cast<const ProjectedStateSpace *>(space), std::move(sampler))
  , constraint_(space->getConstraint()) {
    // check if multiple (constraintIntersection) or single constraint
    const ConstraintIntersectionTransitionPtr cIP = std::dynamic_pointer_cast<ConstraintIntersectionTransition>(
            space->getConstraint());
    if (cIP != nullptr) {
        constraintsVec = cIP->getConstraintsVec();
    }
}

void ompl::base::ProjectedStateSamplerTransition::sampleUniform(State *state)
{
    WrapperStateSampler::sampleUniform(state);
    for(uint i = 0; i < constraintsVec.size(); i++){

        ConstraintPtr cPi = constraintsVec.at(i);

        // check if constraints in vector are transitionConstraints
        const TransitionConstraintPtr tCP = std::dynamic_pointer_cast<TransitionConstraint>(cPi);
        if (tCP != nullptr){

            int newMode = randomNumberGenerator.uniformInt(0,2);

            // std::cout << "Random Mode" << newMode << std::endl;
            tCP->setMode(newMode);
        }
    }
    constraint_->project(state);
    space_->enforceBounds(state);
}

/// ProjectedStateSpaceTransition

/// Public

bool ompl::base::ProjectedStateSpaceTransition::discreteGeodesic(const State *from, const State *to, bool interpolate,
                                                       std::vector<State *> *geodesic) const
{
    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

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

