#ifndef ORTHOKLAMPT_CONTACTCONSTRAINT_H
#define ORTHOKLAMPT_CONTACTCONSTRAINT_H
#endif //ORTHOKLAMPT_CONTACTCONSTRAINT_H


#include <ompl/base/Constraint.h>
#include "planner/cspace/cspace_geometric.h"


class ContactConstraint : public ob::Constraint
{
protected:
    std::vector<Triangle3D> trisFiltered;
    std::vector<Vector2> cornerCoord;

public:
    ContactConstraint(Robot *robot, RobotWorld *world, int robot_idx);


    Vector3 getXYZ(const Eigen::VectorXd xd) const;
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
    Robot *robot_;
    RobotWorld *world_;
    int robot_idx_;
};