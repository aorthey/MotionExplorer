#pragma once

#include "planner/planner.h"
#include "planner/planner_input.h"
#include <Modeling/World.h> //RobotWorld

class MotionPlannerBenchmark: public MotionPlanner{
  public:

    explicit MotionPlannerBenchmark(RobotWorld *world_, PlannerMultiInput& input_);

    virtual void Expand() override;
    virtual void Collapse() override;
    virtual void Next() override;
    virtual void Previous() override;

    virtual std::string getName() const override;
  protected:
    PlannerMultiInput multi_input;

};

