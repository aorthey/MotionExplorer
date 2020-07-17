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
    enum Mode {
      ACTIVE_CONSTRAINT_INITIAL = 0,
      NO_ACTIVE_CONSTRAINT = 1,
      ACTIVE_CONSTRAINT_GOAL = 2
    };
    Mode mode{ACTIVE_CONSTRAINT_INITIAL};

public:
    TransitionConstraint(GeometricCSpaceOMPLRCONTACT *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber, uint obstacleNumber);
    // the 2 contact surfaces as parameters?

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void setMode(int newMode);
    int getMode();


};
