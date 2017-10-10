#pragma once
#include "strategy_input.h"
#include "strategy_output.h"
#include "cspace.h"

class Strategy{
  public:
    virtual void plan( const StrategyInput &input, CSpaceOMPL *cspace, StrategyOutput &output) = 0;

  protected:
    Strategy();
};

