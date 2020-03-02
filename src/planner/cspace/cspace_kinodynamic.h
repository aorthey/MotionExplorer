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
    virtual void initSpace() override;
    virtual void initControlSpace();
    virtual void print(std::ostream& out = std::cout) const override;

    //############################################################################
    //Mapping Functions OMPL <--> KLAMPT
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual void ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl) override;
    virtual ob::ScopedState<> ConfigVelocityToOMPLState(const Config &q, const Config &dq) override;

    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual Config OMPLStateToVelocity(const ob::State *qompl);
    //############################################################################

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
    virtual bool isDynamic() const override;

    virtual Vector3 getXYZ(const ob::State*) override;
};

