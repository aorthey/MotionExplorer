#pragma once
#include "planner/cspace/cspace.h"

//GeometricCSpaceOMPL: Space = Configuration manifold; control space = tangent
//space of configuration manifold, i.e. control happens in velocity space
//
class GeometricCSpaceOMPL: public CSpaceOMPL
{
  public:
    GeometricCSpaceOMPL(RobotWorld *world_, int robot_idx);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si) override;

    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState);

    virtual void print() const override;
};

