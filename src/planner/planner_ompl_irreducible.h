#pragma once

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
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
#include "planner/planner_ompl.h"
#include "cspace_sentinel.h"
#include "util.h"


class MotionPlannerOMPLIrreducible: public MotionPlannerOMPL
{
  public:
    MotionPlannerOMPLIrreducible(RobotWorld *world);
    virtual bool solve(Config &p_init, Config &p_goal);
};


class SentinelPropagatorIrreducible : public SentinelPropagator
{
public:

    SentinelPropagatorIrreducible(oc::SpaceInformationPtr si, KinodynamicCSpaceSentinelAdaptor *cspace) : 
        SentinelPropagator(si,cspace)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

};

