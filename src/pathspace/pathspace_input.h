#pragma once
#include "planner/strategy/strategy_input.h"
#include "planner/cspace/cspace_input.h"
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

class PathSpaceInput{

  public:
    PathSpaceInput(){
      next_layer = NULL;
    }

    Config q_init;
    Config q_goal;
    Config dq_init;
    Config dq_goal;

    Config qMin;
    Config qMax;

    uint robot_idx;
    uint robot_inner_idx;
    uint robot_outer_idx;

    int freeFloating;

    Config se3min;
    Config se3max;

    std::string name_algorithm;

    std::string type;
    uint level;

    double epsilon_goalregion;
    double max_planning_time;
    double timestep_min;
    double timestep_max;

    const CSpaceInput& GetCSpaceInput(){
      cin = new CSpaceInput();
      cin->timestep_max = timestep_max;
      cin->timestep_min = timestep_min;
      return *cin;
    }
    const StrategyInput& GetStrategyInput(){
      sin = new StrategyInput();
      sin->q_init = q_init;
      sin->q_goal = q_goal;
      sin->name_algorithm = name_algorithm;
      sin->epsilon_goalregion = epsilon_goalregion;
      sin->max_planning_time = max_planning_time;
      return *sin;
    }
    PathSpaceInput* GetNextLayer(){
      return next_layer;
    }
    void SetNextLayer(PathSpaceInput* next){
      next_layer = next;
    }

  private:
    CSpaceInput* cin;
    StrategyInput* sin;
    PathSpaceInput* next_layer;
};
