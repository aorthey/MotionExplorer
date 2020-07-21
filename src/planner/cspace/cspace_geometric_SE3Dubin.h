#pragma once
#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLSE3Dubin: public GeometricCSpaceOMPL
{
  using BaseT = GeometricCSpaceOMPL;
  public:
    GeometricCSpaceOMPLSE3Dubin(RobotWorld *world_, int robot_idx);

    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
  private:
    double turningRadius_;
    double climbingAngle_;
};

