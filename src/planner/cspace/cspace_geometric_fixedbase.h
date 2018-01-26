#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLFixedBase: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLFixedBase(RobotWorld *world_, int robot_id, uint N_ = 0);
    virtual void initSpace();
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
  protected:
    uint N;
};

