#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLRN: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLRN(RobotWorld *world_, int robot_id, int dimension);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl);
    virtual void print() const override;
  protected:
    uint N;
};

