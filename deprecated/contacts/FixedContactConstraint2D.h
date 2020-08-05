#pragma once
#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

/*****************************************************
 * Contact Constraint
 * -> for a link that doesn't break contact with initial contact surface.
 *
 * @param cspace
 * @param robot
 * @param world
 * @param linkNumber
 * @param obstacleNumber
 * ***************************************************/

class GeometricCSpaceContact2D;

class FixedContactConstraint2D : public ob::Constraint
{
protected:
    std::vector<Triangle3D> tris;

public:
    FixedContactConstraint2D(GeometricCSpaceContact2D *cspace, int ambientSpaceDim, Robot *robot,RobotWorld *world,
            int linkNumber, std::string meshFrom);


    Vector3 getPos(const Eigen::Ref<const Eigen::VectorXd> &xd) const;
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
    GeometricCSpaceContact2D *cspace_;
    Robot *robot_;
    int linkNumber_;
    std::string  meshFrom_;
};
