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

    constraintIntersection_->project(state);

    space_->enforceBounds(state);

    constraintIntersection_->engraveConstraintMode(state);
    // std::cout << "Random Sample" << std::endl;
    // space_->printState(state);

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

void ompl::base::ProjectedStateSpaceMultiMode::printState(
    const State *state, std::ostream &out) const
{
  out << "Constraint State " << std::endl << "[" << std::endl;
  out << "  ";
  BaseT::printState(state, out);
  if (state != nullptr)
  {
    out << "  Active Contacts ";
    ConstraintMode cfrom = state->as<StateType>()->getMode();

    const std::vector<int>& vecIdx = cfrom.getFixedConstraintIndices();
    const std::vector<Math3D::Vector3>& vec = cfrom.getFixedConstraintContactPoints();

    for(uint k = 0; k < vec.size(); k++)
    {
      Math3D::Vector3 vk = vec.at(k);
      out << "[" << vecIdx.at(k) << ":" << vk << "] ";
    }
    out << std::endl;

    out << "  Constraint Modes: " << cfrom << std::endl;
  }else{
    out << "nullptr" << std::endl;
  }
  out << ']' << std::endl;
}

bool ompl::base::ProjectedStateSpaceMultiMode::discreteGeodesic(
  const State *from, const State *to, bool interpolate, std::vector<State *> *geodesic) const
{
    // std::cout << std::string(80, '-') << std::endl;
    // std::cout << "DISCRETE GEODESIC" << std::endl;
    // std::cout << std::string(80, '-') << std::endl;
    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

    const ConstraintMode& cfrom = from->as<StateType>()->getMode();
    const ConstraintMode& cto = to->as<StateType>()->getMode();
    ConstraintIntersectionMultiModePtr constraintIntersection
      = std::static_pointer_cast<ConstraintIntersectionMultiMode>(constraint_);

    if(!cfrom.canReach(cto))
    {
      return false;
    }

    // State *fromProjected = si_->cloneState(from);
    //if(cto.isLargerAs(cfrom))
    //{
    //  //removing constraint
    //    constraintIntersection->setMode(cto);
    //    // constraint_->project(toProjected);
    //    // constraintIntersection->engraveConstraintMode(toProjected);
    //}else
    //{
    //    //on constraint or adding constraint (at end)
    //    constraintIntersection->setMode(cfrom);
    //    // constraint_->project(toProjected);
    //    // constraintIntersection->engraveConstraintMode(toProjected);
    //}

    constraintIntersection->setTransitionMode(cfrom, cto);

    State *toProjected = si_->cloneState(to);
    constraint_->project(toProjected);
    constraintIntersection->engraveConstraintMode(toProjected);

    // State *toProjected = si_->cloneState(to);

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Trying to connect states" << std::endl;
    si_->printState(from);
    si_->printState(toProjected);

    const double tolerance = delta_;

    // No need to traverse the manifold if we are already there.
    double dist, step, total = 0;
    if ((dist = distance(from, toProjected)) <= tolerance)
    {
        return true;
    }

    const double max = dist * lambda_;

    auto previous = cloneState(from);
    auto scratch = allocState();

    scratch->as<StateType>()->setMode(from->as<StateType>()->getMode());

    if(!constraintIntersection->getMode().isValid())
    {
      std::cout << "INVALID" << std::endl;
    }

    auto &&svc = si_->getStateValidityChecker();

    do
    {
        WrapperStateSpace::interpolate(previous, toProjected, delta_ / dist, scratch);

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
        const double newDist = distance(scratch, toProjected);
        if (newDist >= dist)
            break;

        dist = newDist;
        copyState(previous, scratch);

        // Store the new state
        if (geodesic != nullptr)
        {
            // si_->printState(scratch);
            geodesic->push_back(cloneState(scratch));
        }

    } while (dist >= tolerance);

    std::cout << "Geodesic distance to goal: " << dist << "/" << tolerance<< std::endl;
    freeState(scratch);
    freeState(previous);
    freeState(toProjected);

    return dist <= tolerance;
}
// unsigned int ompl::base::ProjectedStateSpaceMultiMode::validSegmentCount(const State *state1, const State *state2) const
// {
//   return 1;
// }

