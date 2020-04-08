#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLFixedBase: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLFixedBase(RobotWorld *world_, int robot_id);
    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual void print(std::ostream& out = std::cout) const;
    virtual Vector3 getXYZ(const ob::State*) override;
  protected:
    uint N;
};

