#pragma once
#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/ConstrainedSpaceInformation.h>
#include "planner/cspace/contact/ConstraintIntersectionMultiMode.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>


class GeometricCSpaceContact: public GeometricCSpaceOMPL
{

public:
    GeometricCSpaceContact(RobotWorld *world_, int robot_idx);

    std::vector<Triangle3D> getTrianglesOnMesh(std::string nameMesh);

    ob::ConstraintPtr getConstraints();

    void setGoalConstraints();

    void setInitialConstraints();

    void addConstraintsToState(ob::State*);

    virtual Config EigenVectorToConfig(const Eigen::VectorXd &xd) const = 0;

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;

    void initConstraints(ob::StateSpacePtr);

protected:
    std::vector<ob::ConstraintPtr> constraints;
    // ob::ConstraintIntersectionPtr  constraint_intersect;
    ConstraintIntersectionMultiModePtr constraint_intersect;

};
