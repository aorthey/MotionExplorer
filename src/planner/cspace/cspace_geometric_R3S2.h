#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLR3S2: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLR3S2(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual Vector3 getXYZ(const ob::State*) override;
};

