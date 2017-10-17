#pragma once
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/cspace/cspace.h"

class Strategy{
  public:
    virtual void plan( const StrategyInput &input, CSpaceOMPL *cspace, StrategyOutput &output) = 0;

  protected:
    Strategy();
};

