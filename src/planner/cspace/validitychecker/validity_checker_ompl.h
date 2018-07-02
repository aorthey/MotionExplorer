#pragma once
#include "planner/cspace/cspace.h"

class OMPLValidityChecker: public ob::StateValidityChecker
{
  public:
    OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpace *inner_);
    virtual bool isValid(const ob::State* state) const override;
    bool IsCollisionFree(SingleRobotCSpace *space, Config q) const;

    virtual bool IsSufficient(const ob::State* state) const;
    virtual bool IsNecessary(const ob::State* state) const;
    double Distance(const ob::State* state) const;

    CSpaceOMPL* GetCSpacePtr() const;

  protected:
    CSpaceOMPL *cspace;
    CSpace *inner;
};

class OMPLValidityCheckerInnerOuter: public OMPLValidityChecker
{
  public:
    OMPLValidityCheckerInnerOuter(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_, CSpace *outer_);
    virtual bool isValid(const ob::State* state) const;

    CSpace *outer;
};

class OMPLValidityCheckerNecessarySufficient: public OMPLValidityChecker
{
  public:
    OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *outer_);
    virtual bool IsSufficient(const ob::State* state) const;

    CSpace *outer;
};
typedef std::shared_ptr<OMPLValidityChecker> OMPLValidityCheckerPtr;
