#pragma once
#include "planner/strategy/strategy.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
namespace ot = ompl::tools;

class StrategyKinodynamicMultiLevel: public Strategy{
  public:
    virtual void plan( const StrategyInput &input, StrategyOutput &output);

    StrategyKinodynamicMultiLevel();

    ob::PlannerPtr GetPlanner(std::string algorithm,
        std::vector<oc::SpaceInformationPtr> si_vec, 
        std::vector<ob::ProblemDefinitionPtr> pdef_vec);
    void RunBenchmark(
        const StrategyInput& input,
        std::vector<oc::SpaceInformationPtr> si_vec, 
        std::vector<ob::ProblemDefinitionPtr> pdef_vec);

};
