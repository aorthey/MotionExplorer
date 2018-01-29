#pragma once
#include "planner/cspace/cspace.h"

//GeometricCSpaceOMPL: Space = Configuration manifold; control space = tangent
//space of configuration manifold, i.e. control happens in velocity space
//
class GeometricCSpaceOMPL: public CSpaceOMPL
{
  public:
    GeometricCSpaceOMPL(RobotWorld *world_, int robot_idx);
    GeometricCSpaceOMPL(Robot *robot_, CSpaceKlampt *kspace_);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) override;

    void Init();
    virtual void initSpace() override;
    virtual void initControlSpace() override;
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q) override;

    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState);
    virtual void print() const override;
  protected:
    //virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(oc::SpaceInformationPtr si) override;
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si) override;
};

