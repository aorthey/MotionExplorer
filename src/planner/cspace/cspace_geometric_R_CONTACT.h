#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLRCONTACT: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx);
    virtual void initSpace() override; //okay to add "override"?
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print() const override;
};

