#pragma once
#include "planner/cspace/cspace_geometric_contact.h"


class GeometricCSpaceContact2D: public GeometricCSpaceContact
{

public:
    GeometricCSpaceContact2D(RobotWorld *world_, int robot_idx);


    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual Config EigenVectorToConfig(const Eigen::VectorXd &xd) const override;
    virtual void print(std::ostream& out = std::cout) const override;

    virtual Vector3 getXYZ(const ob::State*) override;
};
