#pragma once
#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

class GeometricCSpaceOMPLRCONTACT_3D;

class ContactConstraint_3D : public ob::Constraint
{
protected:
    std::vector<Triangle3D> trisFiltered;
    std::vector<Vector2> cornerCoord;

public:
    ContactConstraint_3D(GeometricCSpaceOMPLRCONTACT_3D *cspace, Robot *robot, RobotWorld *world);


    Vector3 getPos(const Eigen::Ref<const Eigen::VectorXd> &xd, int linkNumber) const;
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;


private:
    GeometricCSpaceOMPLRCONTACT_3D *cspace_;
    Robot *robot_;
    RobotWorld *world_;
};