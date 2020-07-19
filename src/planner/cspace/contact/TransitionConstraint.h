#pragma once
#include "planner/cspace/contact/ContactConstraint.h"
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

/*****************************************************
 * Transition Contact Constraint
 * -> link can either in contact with initial surface, unconstrained/in transition or in contact with different surface
 *
 * @param cspace
 * @param robot
 * @param world
 * @param linkNumber
 * @param obstacleNumber currently doesn't do anything
 * ***************************************************/

OMPL_CLASS_FORWARD(TransitionConstraint);

class GeometricCSpaceOMPLRCONTACT;

class TransitionConstraint : public ContactConstraint
{
protected:
    std::vector<Triangle3D> trisFiltered;
    std::vector<Triangle3D> trisFiltered_negative;
    uint mode;

public:
    TransitionConstraint(GeometricCSpaceOMPLRCONTACT *cspace, Robot *robot, RobotWorld *world, uint linkNumber, uint obstacleNumber);


    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void setMode(uint newMode);


};