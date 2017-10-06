#pragma once
#include "planner_input.h"
#include "planner_output.h"
#include "cspace.h"

class Algorithm{
  public:
    virtual void plan( const PlannerInput &input, CSpaceOMPL *cspace, PlannerOutput &output) = 0;

  protected:
    Algorithm();
};

