#pragma once
#include "planner/strategy/strategy.h"
#include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
namespace ot = ompl::tools;

class StrategyGeometricMultiLevel: public Strategy{
  public:
    virtual void plan( const StrategyInput &input, StrategyOutput &output);

    StrategyGeometricMultiLevel();

    ob::PlannerPtr GetPlanner(std::string algorithm,
        std::vector<ob::SpaceInformationPtr> si_vec, 
        std::vector<ob::ProblemDefinitionPtr> pdef_vec);
    void RunBenchmark(
        const StrategyInput& input,
        std::vector<ob::SpaceInformationPtr> si_vec, 
        std::vector<ob::ProblemDefinitionPtr> pdef_vec);

};

