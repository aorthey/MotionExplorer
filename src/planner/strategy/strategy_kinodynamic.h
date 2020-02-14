#pragma once
#include "planner/strategy/strategy.h"
//#include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

class StrategyKinodynamicMultiLevel: public Strategy{
  public:
    //virtual void plan( const StrategyInput &input, StrategyOutput &output);
    virtual void Plan( StrategyOutput &output) override;
    virtual void Step( StrategyOutput &output) override;
    virtual void Init( const StrategyInput &input) override;
    virtual void Clear() override;

    StrategyKinodynamicMultiLevel();

    ob::PlannerPtr GetPlanner(std::string algorithm,
        std::vector<ob::SpaceInformationPtr> si_vec, 
        ob::ProblemDefinitionPtr pdef,
        const StrategyInput& input);
    // void RunBenchmark(
    //     const StrategyInput& input,
    //     std::vector<oc::SpaceInformationPtr> si_vec, 
    //     std::vector<ob::ProblemDefinitionPtr> pdef_vec);

};
