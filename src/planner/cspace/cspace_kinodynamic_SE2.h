#pragma once
#include "planner/cspace/cspace_kinodynamic.h"

class KinodynamicCSpaceOMPLSE2: public KinodynamicCSpaceOMPL
{
  public:
    KinodynamicCSpaceOMPLSE2(RobotWorld *world_, int robot_idx);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual void initSpace() override;
    virtual void initControlSpace() override;
    virtual void print(std::ostream& out = std::cout) const override;

    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual void ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl) override;
    ob::ScopedState<> ConfigVelocityToOMPLState(const Config &q, const Config &dq) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual Config OMPLStateToVelocity(const ob::State *qompl) override;

};

