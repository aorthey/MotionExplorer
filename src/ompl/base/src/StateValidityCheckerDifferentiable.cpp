#include <ompl/base/StateValidityCheckerDifferentiable.h>
#include <ompl/util/Exception.h>

using namespace ompl::base;

StateValidityCheckerDifferentiable::StateValidityCheckerDifferentiable(
    const SpaceInformationPtr &si): BaseT(si)
{
}

StateValidityCheckerDifferentiable::StateValidityCheckerDifferentiable(
    SpaceInformation *si): BaseT(si)
{
}

double StateValidityCheckerDifferentiable::cost(const State *state) const
{
  throw ompl::Exception("NYI");
}

Eigen::VectorXd StateValidityCheckerDifferentiable::costGradient(const State *state) const
{
  throw ompl::Exception("NYI");
}
