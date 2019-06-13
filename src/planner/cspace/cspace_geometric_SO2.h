#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLSO2: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLSO2(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print() const override;
};

