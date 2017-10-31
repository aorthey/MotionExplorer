#include "planner/cspace/cspace.h"

class GeometricCSpaceOMPLR2: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLR2(Robot *robot_, CSpace *space_);
    virtual void initSpace();
    virtual void print();
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
};

