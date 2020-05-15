#include <ompl/geometric/SimpleSetup.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>


class GeometricCSpaceOMPLRCONTACT: public GeometricCSpaceOMPL
{
protected:
    //RobotWorld world;

    ob::ConstraintPtr constraint;
    ob::ConstrainedStateSpacePtr css;

public:
    GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx);

    std::vector<Triangle3D> trisFiltered;
    std::vector<Vector2> cornerCoord;


    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const override;
    virtual Vector3 getXYZ(const ob::State *s);

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
};