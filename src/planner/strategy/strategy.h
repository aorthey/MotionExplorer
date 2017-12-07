#pragma once
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/cspace/cspace.h"



class Strategy{
  public:
    virtual void plan( const StrategyInput &input, StrategyOutput &output) = 0;

  protected:
    Strategy();
    void setStateSampler(std::string sampler, ob::SpaceInformationPtr si);
};

