#include <ompl/geometric/SimpleSetup.h>
#include "planner/cspace/cspace_geometric.h"
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>


class GeometricCSpaceOMPLRCONTACT: public GeometricCSpaceOMPL
{
  protected:
    //Robot *robot{nullptr};
    //RobotWorld *world{nullptr};

    ob::ConstraintPtr constraint;
    ob::ConstrainedStateSpacePtr css;
    // ob::ConstrainedSpaceInformationPtr csi;
    // ompl::geometric::SimpleSetupPtr ss;

  public:
    GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx);

    std::vector<Triangle3D> surf_triangles;

    virtual void initSpace() override;
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const;
    virtual Vector3 getXYZ(const ob::State *s);

    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
    // virtual ob::SpaceInformationPtr SpaceInformationPtr() override;
};


