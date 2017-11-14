#include "planner/cspace/cspace.h"

class GeometricCSpaceOMPLR2EdgeSO2: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLR2EdgeSO2(RobotWorld *world_, int robot_idx, const Config& q_src, const Config& q_trg);
    virtual void initSpace();
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
  private:
    Config q1;
    Config q2;
    double edge_length;
};

