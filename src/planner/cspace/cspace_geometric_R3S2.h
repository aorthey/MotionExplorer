#include "planner/cspace/cspace.h"

class GeometricCSpaceOMPLR3S2: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLR3S2(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
};

