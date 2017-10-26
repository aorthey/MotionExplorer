#include "planner/cspace/cspace.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>

class GeometricCSpaceOMPLSE2: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLSE2(Robot *robot_, CSpace *space_);
    virtual void initSpace();
    virtual void print();
    virtual ob::ScopedState<> ConfigToOMPLState(const Config &q);
    virtual Config OMPLStateToConfig(const ob::State *qompl);
};

