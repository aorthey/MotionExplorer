#pragma once
#include "planner/cspace/contact/ContactConstraint_3D.h"
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

OMPL_CLASS_FORWARD(TransitionConstraint_3D);

class GeometricCSpaceOMPLRCONTACT_3d;

class TransitionConstraint_3D : public ContactConstraint_3D
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
    TransitionConstraint_3D(GeometricCSpaceOMPLRCONTACT_3D *cspace, int ambientSpaceDim, Robot *robot, RobotWorld *world, uint linkNumber);

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void setMode(int newMode);
    int getMode();

};
