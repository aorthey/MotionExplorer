#include <ompl/base/Constraint.h>
#include "planner/cspace/contact/ConstraintMultiMode.h"

ConstraintMultiMode::ConstraintMultiMode(int ambientSpaceDim)
  : ompl::base::Constraint(ambientSpaceDim, 1)
{
}
void ConstraintMultiMode::setMode(int mode)
{
  if(mode <= this->getNumberOfModes())
  {
    mode_ = mode;
  }
}
int ConstraintMultiMode::setRandomMode()
{
    int rMax = getNumberOfModes() - 1;
    int r = randomNumberGenerator_.uniformInt(0, rMax);
    setMode(r);
    return r;
}
int ConstraintMultiMode::getMode() const
{
  return mode_;
}
