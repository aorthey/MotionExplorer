#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <boost/program_options.hpp>
#include "planner.h"
#include "util.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace po = boost::program_options;

class MotionPlannerOMPL: public MotionPlanner
{
  public:
    MotionPlannerOMPL(RobotWorld *world, WorldSimulation *sim);
    ob::ScopedState<> ConfigToOMPLState(Config &q, ob::StateSpacePtr &s);
    Config OMPLStateToConfig(ob::ScopedState<> &qompl, ob::StateSpacePtr &s);
    Config OMPLStateToConfig(ob::State *qompl, ob::StateSpacePtr &s);
    Config OMPLStateToConfig(ob::SE3StateSpace::StateType *qomplSE3, ob::RealVectorStateSpace::StateType *qomplRnState, ob::StateSpacePtr &s);
    void test();
    void test_conversion(Config &q, ob::StateSpacePtr &stateSpace);
    virtual bool solve(Config &p_init, Config &p_goal);
};

