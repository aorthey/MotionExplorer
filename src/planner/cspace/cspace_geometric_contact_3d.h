#pragma once
#include "planner/cspace/cspace_geometric_contact.h"
#include "planner/cspace/contact/ProjectedStateSpace_Transition.h"
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>


class GeometricCSpaceContact3D: public GeometricCSpaceContact
{
public:
    GeometricCSpaceContact3D(RobotWorld *world_, int robot_idx);


    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config EigenVectorToConfig(const Eigen::VectorXd &xd) const;
    virtual void print(std::ostream& out = std::cout) const override;

    virtual Vector3 getXYZ(const ob::State*) override;
};
