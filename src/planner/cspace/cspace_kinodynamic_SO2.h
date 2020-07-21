#pragma once
#include "planner/cspace/cspace_kinodynamic.h"

class KinodynamicCSpaceOMPLSO2: public KinodynamicCSpaceOMPL
{
  public:
    KinodynamicCSpaceOMPLSO2(RobotWorld *world_, int robot_idx);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual void initSpace() override;
    virtual void initControlSpace() override;

    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual void ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual Config OMPLStateToVelocity(const ob::State *qompl) override;

    virtual Vector3 getXYZ(const ob::State*) override;

    virtual Config ControlToConfig(const double*) override;
};

