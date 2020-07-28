#pragma once
#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/ConstrainedSpaceInformation.h>
#include "planner/cspace/contact/ProjectedStateSpace_Transition.h"
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include "planner/cspace/contact/TransitionModeTypes.h"


class GeometricCSpaceContact: public GeometricCSpaceOMPL
{
protected:
    std::vector<ob::ConstraintPtr> constraints;
    ob::ConstraintIntersectionPtr  constraint_intersect;

public:
    GeometricCSpaceContact(RobotWorld *world_, int robot_idx);

    void setGoalConstraints();
    void setInitialConstraints();
    void setConstraintsMode(TransitionMode mode);

    virtual void initSpace() = 0;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) = 0;
    virtual Config OMPLStateToConfig(const ob::State *qompl) = 0;

    virtual Vector3 getXYZ(const ob::State*) = 0;

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
};
