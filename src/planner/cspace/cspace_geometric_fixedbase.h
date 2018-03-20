#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLFixedBase: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLFixedBase(RobotWorld *world_, int robot_id);
    virtual void initSpace();
    void Init();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual void print() const override;
  protected:
    uint N;
};

