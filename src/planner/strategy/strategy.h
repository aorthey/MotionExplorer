#pragma once
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/base/spaces/SE3StateSpace.h>

class Strategy{
  public:
    virtual void Plan( StrategyOutput &output) = 0;
    virtual void Step( StrategyOutput &output) = 0;
    virtual void Init( const StrategyInput &input) = 0;
    virtual void Clear() = 0;

    void BenchmarkFileToPNG(const std::string&);

    bool IsInitialized();

  protected:
    Strategy() = default;
    void setStateSampler(std::string sampler, ob::SpaceInformationPtr si);

    bool isInitialized{false};

    ob::PlannerPtr planner;

    double max_planning_time;
};
