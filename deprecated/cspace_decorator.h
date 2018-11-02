#pragma once
#include "planner/cspace/cspace.h"

class CSpaceOMPLDecorator: public CSpaceOMPL
{
  public:
    CSpaceOMPLDecorator(CSpaceOMPL*);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) override;
    virtual void initSpace() override;
    virtual void print() const override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;

    using CSpaceOMPL::StateValidityCheckerPtr;
    using CSpaceOMPL::getXYZ;
    using CSpaceOMPL::SetCSpaceInput;
    using CSpaceOMPL::GetDimensionality;
    using CSpaceOMPL::GetKlamptDimensionality;
    using CSpaceOMPL::GetControlDimensionality;
    using CSpaceOMPL::OMPLStateToConfig;
    using CSpaceOMPL::GetRobotPtr;
    using CSpaceOMPL::GetWorldPtr;
    using CSpaceOMPL::GetCSpacePtr;
    using CSpaceOMPL::SpacePtr;
    using CSpaceOMPL::SpaceInformationPtr;
    using CSpaceOMPL::ControlSpacePtr;

    using CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace;
    using CSpaceOMPL::OMPLSO3StateSpaceFromEulerXYZ;
    using CSpaceOMPL::isDynamic;
    using CSpaceOMPL::isFixedBase;
    using CSpaceOMPL::isFreeFloating;
    using CSpaceOMPL::print;

  protected:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override = 0;
    CSpaceOMPL *cspace_ompl;

};

class CSpaceOMPLDecoratorInnerOuter: public CSpaceOMPLDecorator
{
  public:
    CSpaceOMPLDecoratorInnerOuter(CSpaceOMPL *_cspace_, CSpace *outer_);
  private:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override;
    CSpace *outer;
};

class CSpaceOMPLDecoratorNecessarySufficient: public CSpaceOMPLDecorator
{
  public:
    //CSpaceOMPLDecoratorNecessarySufficient(CSpaceOMPL *_cspace_, CSpace *outer_);
    CSpaceOMPLDecoratorNecessarySufficient(CSpaceOMPL *_cspace_, uint robot_outer_idx_);
  private:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override;
    CSpace *outer;
    uint robot_outer_idx;
};
