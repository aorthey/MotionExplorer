#include "planner/cspace/contact/ProjectedStateSpaceMultiMode.h"
#include "planner/cspace/contact/ConstraintIntersectionMultiMode.h"
#include <utility>

/// ProjectedStateSampler
/// Public

ompl::base::ProjectedStateSamplerMultiMode::ProjectedStateSamplerMultiMode(const ProjectedStateSpaceMultiMode *space, StateSamplerPtr sampler)
  : ProjectedStateSampler(reinterpret_cast<const ProjectedStateSpace *>(space), std::move(sampler))
  , constraint_(space->getConstraint()) 
{
    // check if multiple (constraintIntersection) or single constraint
    constraintIntersection_ =
      std::dynamic_pointer_cast<ConstraintIntersectionMultiMode>(constraint_);
    if (constraintIntersection_ != nullptr) 
    {
        constraintsVec = constraintIntersection_->getConstraintsVec();
    }
}

void ompl::base::ProjectedStateSamplerMultiMode::sampleUniform(State *state)
{
    WrapperStateSampler::sampleUniform(state);

    constraintIntersection_->setRandomMode();

    state->as<ProjectedStateSpaceMultiMode::StateType>()->setMode(constraintIntersection_->getMode());
    
    constraintIntersection_->project(state);

    space_->enforceBounds(state);
}

void ompl::base::ProjectedStateSamplerMultiMode::sampleUniformNear(
    State *state, const State *near, double distance)
{
  OMPL_ERROR("NYI");
  ompl::Exception("NYI");
}
void ompl::base::ProjectedStateSamplerMultiMode::sampleGaussian(
    State *state, const State *mean, double stdDev)
{
  OMPL_ERROR("NYI");
  ompl::Exception("NYI");
}

bool ompl::base::ProjectedStateSpaceMultiMode::discreteGeodesic(
  const State *from, const State *to, bool interpolate, std::vector<State *> *geodesic) const
{
    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

    ConstraintMode cfrom = from->as<StateType>()->getMode();
    ConstraintMode cto = to->as<StateType>()->getMode();

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Trying to connect states" << std::endl;
    si_->printState(from);
    std::cout << "Mode:" << cfrom << std::endl;
    si_->printState(to);
    std::cout << "Mode:" << cto << std::endl;

    if(!cfrom.canReach(cto))
    {
      std::cout << "Cannot be reached" << std::endl;
      return false;
    }

    ConstraintIntersectionMultiModePtr constraintIntersection
      = std::static_pointer_cast<ConstraintIntersectionMultiMode>(constraint_);

    if(cfrom.isLargerAs(cto))
    {
        std::cout << "From InActive to Active" << std::endl;
        constraintIntersection->setMode(cfrom);
    }else{
        std::cout << "From Active to Inactive" << std::endl;
        constraintIntersection->setMode(cto);
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
        {
            geodesic->push_back(cloneState(scratch));
        }

    } while (dist >= tolerance);

    std::cout << "Geodesic distance to goal: " << dist << "/" << tolerance<< std::endl;
    freeState(scratch);
    freeState(previous);

    return dist <= tolerance;
}
// unsigned int ompl::base::ProjectedStateSpaceMultiMode::validSegmentCount(const State *state1, const State *state2) const
// {
//   return 1;
// }

