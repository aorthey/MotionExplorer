#pragma once
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

class OMPLValidityCheckerRelaxation: public OMPLValidityChecker
{
  using BaseT = OMPLValidityChecker;

  public:
    OMPLValidityCheckerRelaxation(const ob::SpaceInformationPtr &si, 
        CSpaceOMPL *cspace, ob::State*, double);

    bool isValid(const ob::State* state) const override;

    virtual bool operator ==(const ob::StateValidityChecker &rhs) const override;

    ob::State *getStateCenter() const;
    double getRadius() const;

  protected:
    ob::State *xCenter_;
    double radius_;
};

typedef std::shared_ptr<OMPLValidityCheckerRelaxation> OMPLValidityCheckerRelaxationPtr;
