#include "planner/cspace/validitychecker/validity_checker_ompl_relaxation.h"
#include "common.h"
#include <ompl/base/StateSpaceTypes.h>

OMPLValidityCheckerRelaxation::OMPLValidityCheckerRelaxation(
    const ob::SpaceInformationPtr &si, 
    CSpaceOMPL *cspace,
    ob::State *xCenter,
    double radius):
  BaseT(si, cspace), xCenter_(xCenter), radius_(radius)
{
}

bool OMPLValidityCheckerRelaxation::isValid(const ob::State* x) const
{
  si_->printState(xCenter_);
  double d = si_->distance(xCenter_, x);
  if( d > radius_) return true;
  else return BaseT::isValid(x);
}

bool OMPLValidityCheckerRelaxation::operator ==(const ob::StateValidityChecker &rhs) const
{
  bool sameSpace = (BaseT::operator==(rhs));

  if(!sameSpace) return false;

  const OMPLValidityCheckerRelaxation* crhs = dynamic_cast<const OMPLValidityCheckerRelaxation*>(&rhs);
  if(crhs == nullptr) return false;
  else{
    return ((xCenter_ == crhs->getStateCenter()) 
        && (radius_ == crhs->getRadius()));
  }
}

ob::State* OMPLValidityCheckerRelaxation::getStateCenter() const
{
  return xCenter_;
}

double OMPLValidityCheckerRelaxation::getRadius() const
{
  return radius_;
}

