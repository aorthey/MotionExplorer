#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
//#include <boost/program_options.hpp>
#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include "planner.h"
#include "util.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
//namespace po = boost::program_options;

ob::ScopedState<> ConfigToOMPLState(const Config &q, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::ScopedState<> &qompl, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::State *qompl, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::StateSpacePtr &s);

class MotionPlannerOMPLValidityChecker : public ob::StateValidityChecker
{
   public:
     MotionPlannerOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace* space);
     virtual bool isValid(const ob::State* state) const;
     CSpace* _space;
};

class MotionPlannerOMPL: public MotionPlanner
{
  public:
    MotionPlannerOMPL(RobotWorld *world, WorldSimulation *sim);
    void test();
    void test_conversion(Config &q, ob::StateSpacePtr &stateSpace);
    virtual bool solve(Config &p_init, Config &p_goal);
};

