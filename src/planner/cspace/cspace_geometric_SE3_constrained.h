#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLSE3Constrained: public GeometricCSpaceOMPL
{
    using BaseT = GeometricCSpaceOMPL;
  public:
    GeometricCSpaceOMPLSE3Constrained(RobotWorld *world_, int robot_idx);

    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState);

    virtual void print(std::ostream& out = std::cout) const;
    virtual bool isDynamic() const override;
    virtual Vector3 getXYZ(const ob::State*) override;
};

