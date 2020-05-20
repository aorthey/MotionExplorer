#pragma once
#include "planner/cspace/cspace_geometric_SE2RN.h"

class GeometricCSpaceOMPLSE2Dubin: public GeometricCSpaceOMPLSE2RN
{
  using BaseT = GeometricCSpaceOMPLSE2RN;
  public:
    GeometricCSpaceOMPLSE2Dubin(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    // virtual void print(std::ostream& out = std::cout) const;
    // virtual Vector3 getXYZ(const ob::State*) override;
    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
  private:
    double turningRadius_;
};

