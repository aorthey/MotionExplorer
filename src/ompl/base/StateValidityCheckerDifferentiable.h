#ifndef OMPL_BASE_STATE_VALIDITY_CHECKER_DIFFERENTIABLE_
#define OMPL_BASE_STATE_VALIDITY_CHECKER_DIFFERENTIABLE_

#include <ompl/base/StateValidityChecker.h>
#include <Eigen/Core>

namespace ompl
{
    namespace base
    {
        class StateValidityCheckerDifferentiable: public StateValidityChecker
        {
              using BaseT = StateValidityChecker;
          public:
              StateValidityCheckerDifferentiable() = delete;

              StateValidityCheckerDifferentiable(const SpaceInformationPtr &si);
              StateValidityCheckerDifferentiable(SpaceInformation *si);
              virtual ~StateValidityCheckerDifferentiable() = default;

              virtual bool isValid(const State *state) const = 0;

              virtual double cost(const State *state) const;

              virtual Eigen::VectorXd costGradient(const State *state) const;
        };
    }
}

#endif
