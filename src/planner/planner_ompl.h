#pragma once

//#include <ompl/base/spaces/DubinsStateSpace.h>
//#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/benchmark/Benchmark.h>

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
#include "util.h"
#include "principalfibrebundle.h"
#include "omplklamptconverter.h"
#include "ompl_space.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
namespace ot = ompl::tools;

class MotionPlannerOMPL: public MotionPlanner
{
  public:
    MotionPlannerOMPL(RobotWorld *world);
    void SerializeTree(ob::PlannerData &pd);
    virtual bool solve(PlannerInput& input);

    static ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
      ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
      obj->setCostThreshold(ob::Cost(dInf));
      return obj;
    }

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
