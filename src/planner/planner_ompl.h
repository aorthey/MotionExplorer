#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/Syclop.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
//#include <boost/program_options.hpp>
#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include "planner/planner.h"
#include "cspace_sentinel.h"
#include "util.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
//namespace po = boost::program_options;

ob::ScopedState<> ConfigToOMPLState(const Config &q, const ob::StateSpacePtr &s);
ob::State* ConfigToOMPLStatePtr(const Config &q, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::ScopedState<> &qompl, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::State *qompl, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::StateSpacePtr &s);

class GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPL(Robot *robot);
    const ob::StateSpacePtr getPtr(){
      return space_;
    }
  protected:
    ob::StateSpacePtr space_;
};

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
    void SerializeTree(ob::PlannerData &pd);
    void test();
    void test_conversion(Config &q, ob::StateSpacePtr &stateSpace);
    virtual bool solve(Config &p_init, Config &p_goal);
};

class SentinelPropagator : public oc::StatePropagator
{
public:

    SentinelPropagator(oc::SpaceInformationPtr si, KinodynamicCSpaceSentinelAdaptor *cspace) : 
        oc::StatePropagator(si.get()), cspace_(cspace)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

    KinodynamicCSpaceSentinelAdaptor *cspace_;

};


class SE3Project0r : public ob::ProjectionEvaluator
{
  public:
    SE3Project0r(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
    {
    }
    virtual unsigned int getDimension(void) const
    {
      return 3;
    }
    virtual void defaultCellSizes(void)
    {
      cellSizes_.resize(3);
      cellSizes_[0] = 0.1;
      cellSizes_[1] = 0.1;
      cellSizes_[2] = 0.1;
    }
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
      const ob::SE3StateSpace::StateType *stateSE3 = state->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
      projection(0) = stateSE3->getX();
      projection(1) = stateSE3->getY();
      projection(2) = stateSE3->getZ();
    }
};
