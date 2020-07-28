#pragma once
#include "planner/cspace/contact/ContactConstraint3D.h"
#include "planner/cspace/contact/TransitionModeTypes.h"
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

/*****************************************************
 * Transition Contact Constraint 3D
 * -> link can either in contact with initial surface, unconstrained/in transition or in contact with different surface
 *
 * @param cspace
 * @param robot
 * @param world
 * @param linkNumber
 * @param obstacleNumber currently doesn't do anything
 * ***************************************************/

class TransitionConstraint3D : public ContactConstraint3D
{
protected:
    std::vector<Triangle3D> trisFrom;
    std::vector<Triangle3D> trisTo;

    TransitionMode mode{ACTIVE_CONSTRAINT_INITIAL};

public:
    TransitionConstraint3D(GeometricCSpaceContact3D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, std::string meshFrom, std::string meshTo);

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void setMode(int newMode);
    int getMode();

};
