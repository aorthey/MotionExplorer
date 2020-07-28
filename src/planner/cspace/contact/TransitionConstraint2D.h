#pragma once
#include "planner/cspace/contact/ContactConstraint2D.h"
#include "planner/cspace/contact/TransitionModeTypes.h"
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

class GeometricCSpaceContact2D;

class TransitionConstraint2D : public ContactConstraint2D
{
protected:
    std::vector<Triangle3D> trisFrom;
    std::vector<Triangle3D> trisTo;

    TransitionMode mode{ACTIVE_CONSTRAINT_INITIAL};

public:
    TransitionConstraint2D(GeometricCSpaceContact2D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, std::string meshFrom, std::string meshTo);

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void setMode(int newMode);
    int getMode();

};
