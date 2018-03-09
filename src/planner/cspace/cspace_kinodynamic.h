#pragma once
#include "planner/cspace/cspace_geometric.h"

//KinodynamicCSpaceOMPL: Space = tangent bundle of configuration manifold;
//control space = tangent space of tangent bundle, i.e. control happens in
//acceleration space, i.e. we can control torques of revolute joints, forces
//of prismatic joints, and any additional booster/thruster which act directly
//on the se(3) component

class KinodynamicCSpaceOMPL: public GeometricCSpaceOMPL
{
  public:
    KinodynamicCSpaceOMPL(RobotWorld *world_, int robot_idx);

    virtual const oc::StatePropagatorPtr StatePropagatorPtr(oc::SpaceInformationPtr si);
    virtual void initSpace();
    virtual void initControlSpace();
    virtual void print() const override;
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual ob::ScopedState<> ConfigVelocityToOMPLState(const Config &q, const Config &dq);
    virtual Config OMPLStateToConfig(const ob::State *qompl);

    ob::SpaceInformationPtr SpaceInformationPtr() override;

    //Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::RealVectorStateSpace::StateType *qomplTMState);

  protected:
    virtual const ob::StateValidityCheckerPtr StateValidityCheckerPtr(ob::SpaceInformationPtr si);
    std::vector<int> vel_ompl_to_klampt;
    std::vector<int> vel_klampt_to_ompl;

};

