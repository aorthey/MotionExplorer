#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLFixedBase: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLFixedBase(RobotWorld *world_, int robot_id);
    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print() const override;
    virtual Vector3 getXYZ(const ob::State *s);
  protected:
    uint N;
};

