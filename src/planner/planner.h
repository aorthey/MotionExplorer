#pragma once

#include "planner/planner_input.h"
#include "planner/planner_output.h"

#include <Modeling/World.h> //RobotWorld
#include <Planning/RobotCSpace.h> //SingleRobotCSpace
#include <tinyxml.h>
#include <vector>

class MotionPlanner{

  protected:

    RobotWorld *world;
    PlannerInput input;
    PlannerOutput output;

  public:

    explicit MotionPlanner(RobotWorld *world_, PlannerInput& input_);

    virtual bool solve();

    PlannerInput GetInput();
    PlannerOutput GetOutput();

    bool IsFeasible(Robot *robot, SingleRobotCSpace &cspace, Config &q);

  private:

};

