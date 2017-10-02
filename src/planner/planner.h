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

    virtual bool solve(){
      std::cout << "NYI" << std::endl;
      exit(1);
    }

    const PlannerInput& GetInput();
    const PlannerOutput& GetOutput();

    bool IsFeasible(Robot *robot, SingleRobotCSpace &cspace, Config &q);

};

