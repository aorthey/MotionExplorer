#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLEmpty: public GeometricCSpaceOMPL
{

  public:

    GeometricCSpaceOMPLEmpty(RobotWorld *world_, int robot_idx);
    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const override;
    virtual Vector3 getXYZ(const ob::State*) override;
    virtual uint GetDimensionality() const override;
    // virtual uint GetKlamptDimensionality() const override;

  protected:

    Config q_;
    uint N;
};

