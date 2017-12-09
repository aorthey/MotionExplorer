#pragma once

#include "planner/planner.h"
#include "planner/planner_input.h"
#include <Modeling/World.h> //RobotWorld

class MotionPlannerBenchmark: public MotionPlanner{
  public:

    explicit MotionPlannerBenchmark(RobotWorld *world_, PlannerMultiInput& input_);

    virtual void Expand();
    virtual void Collapse();
    virtual void Next();
    virtual void Previous();

};

