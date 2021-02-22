#include <ompl/base/goals/GoalStateDifferentiable.h>
#include <ompl/util/Exception.h>
#include <KrisLibrary/math/vector.h>

using namespace ompl::base;

GoalStateDifferentiable::GoalStateDifferentiable(const SpaceInformationPtr &si) 
  : GoalState(si)
{
    neighbor_ = si->allocState();
}

GoalStateDifferentiable::~GoalStateDifferentiable()
{
    si_->freeState(neighbor_);
}

double GoalStateDifferentiable::cost(const State *state) const
{
    return distanceGoal(state);
}

Eigen::VectorXd GoalStateDifferentiable::costGradient(const State *state) const
{
    double d = distanceGoal(state);

    const base::State *goalState = getState();

    double minStep = si_->getStateValidityCheckingResolution();

    double step = minStep / d;

    //NOTE: subtraction q - qGoal is not well defined on general manifold. 
    // We circumvent this problem by doing a small step into direction of qGoal
    // and assume that both states are approximately in the same vector space,
    // so that we can do subtraction. In the future, this should be done
    // rigorously by having a connection on the manifold to relate the tangent
    // spaces to the two states (and then do proper subtraction in the same
    // tangent space).
    si_->getStateSpace()->interpolate(state, goalState, step, neighbor_);

    uint N = si_->getStateDimension();

    std::vector<double> s1d(N);
    std::vector<double> s2d(N);
    si_->getStateSpace()->copyToReals(s1d, state);
    si_->getStateSpace()->copyToReals(s2d, neighbor_);

    Math::Vector s1 = s1d;
    Math::Vector s2 = s2d;
    Math::Vector direction = (s2 - s1);
    double dn = direction.norm();
    direction = direction / dn;

    double zeta = 1.0;
    double dGoalStar = 1.0; //switch between functions

    Math::Vector force;
    if(dn <= dGoalStar)
    {
      force = zeta*d*direction;
    }else{
      force = dGoalStar*zeta*direction;
    }
    Eigen::VectorXd dx(N);
    for(uint k = 0; k < N; k++)
    {
      dx(k) = force[k];
    }
    return dx;
}

