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

    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr() override;
    virtual Vector3 getXYZ(const ob::State*) override;
    virtual void SetCSpaceInput(const CSpaceInput &input_) override;
    virtual uint GetDimensionality() const override;
    virtual uint GetControlDimensionality() const override;
    virtual Config OMPLStateToConfig(const ob::ScopedState<> &qompl) override;
    virtual Robot* GetRobotPtr() override;
    virtual CSpaceKlampt* GetCSpacePtr() override;
    virtual const ob::StateSpacePtr SpacePtr() override;
    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
    virtual const oc::RealVectorControlSpacePtr ControlSpacePtr() override;
    virtual void print(std::ostream& out) const override;

  protected:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override;
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
