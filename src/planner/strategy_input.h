#pragma once
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct StrategyInput{
  Config q_init;
  Config q_goal;
  std::string name_algorithm;
  double epsilon_goalregion;
  double max_planning_time;
};
