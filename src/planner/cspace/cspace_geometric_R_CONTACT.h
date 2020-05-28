#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>


class GeometricCSpaceOMPLRCONTACT: public GeometricCSpaceOMPL
{
protected:
    ob::ConstraintPtr constraint;
    ob::ConstrainedStateSpacePtr css;

public:
    GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx);


    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    Config EigenVectorToConfig(const Eigen::VectorXd &xd) const;
    virtual void print(std::ostream& out = std::cout) const override;

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
};