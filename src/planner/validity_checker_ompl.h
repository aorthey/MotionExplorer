#pragma once
#include "cspace.h"

class OMPLValidityChecker: public ob::StateValidityChecker
{
  public:
    OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_);
    virtual bool isValid(const ob::State* state) const;
    bool isCollisionFree(SingleRobotCSpace *space, Config q) const;

    virtual bool isSufficient(const ob::State* state) const;
    virtual bool isNecessary(const ob::State* state) const;

    CSpaceOMPL *ompl_space;
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
    OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_, CSpace *outer_);
    virtual bool isValid(const ob::State* state) const;
    virtual bool isSufficient(const ob::State* state) const;
    virtual bool isNecessary(const ob::State* state) const;

    CSpace *outer;
};
typedef std::shared_ptr<OMPLValidityChecker> OMPLValidityCheckerPtr;
