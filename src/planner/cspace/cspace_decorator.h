#pragma once
#include "planner/cspace/cspace.h"

class CSpaceOMPLDecorator: public CSpaceOMPL
{
  public:
    CSpaceOMPLDecorator(CSpaceOMPL*);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);

    virtual void initSpace();
    virtual void initControlSpace();
    virtual void print();

    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);

    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual const ob::StateSpacePtr SpacePtr();
    virtual const oc::RealVectorControlSpacePtr ControlSpacePtr();
    virtual uint GetDimensionality();
    virtual uint GetControlDimensionality();
    virtual void SetCSpaceInput(CSpaceInput &input_);
    virtual Robot* GetRobotPtr();
    virtual CSpace* GetCSpacePtr();
  protected:
    CSpaceOMPL *cspace_ompl;
};

class CSpaceOMPLDecoratorInnerOuter: public CSpaceOMPLDecorator
{
  public:
    CSpaceOMPLDecoratorInnerOuter(CSpaceOMPL *_cspace_, CSpace *outer_);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);
  private:
    CSpace *outer;
};

class CSpaceOMPLDecoratorNecessarySufficient: public CSpaceOMPLDecorator
{
  public:
    CSpaceOMPLDecoratorNecessarySufficient(CSpaceOMPL *_cspace_, CSpace *outer_);
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);
  private:
    CSpace *outer;
};
