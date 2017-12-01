#include "planner/cspace/cspace.h"

class GeometricCSpaceOMPLRN: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLRN(RobotWorld *world_, int robot_id, int dimension);
    virtual void initSpace();
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
  protected:
    uint N;
};

