#pragma once
#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

/*****************************************************
 * Contact Constraint in 3D
 * -> contact with closest object surface
 *
 * @param cspace
 * @param robot
 * @param world
 * ***************************************************/

class GeometricCSpaceContact3D;

class ContactConstraint3D : public ob::Constraint
{
protected:
    std::vector<Triangle3D> tris;

public:
    ContactConstraint3D(GeometricCSpaceContact3D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, int linkNumber, std::string meshFrom);


    Vector3 getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const;
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
    GeometricCSpaceContact3D *cspace_;
    Robot *robot_;
    RobotWorld *world_;
    int linkNumber_;
    std::string meshFrom_;
};
