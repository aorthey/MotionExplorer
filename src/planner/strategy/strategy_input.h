#pragma once
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct StrategyInput{
  Config q_init;
  Config q_goal;
  std::string name_algorithm;
  double max_planning_time;

  double epsilon_goalregion;
  std::vector<double> epsilon_goalregion_per_dimension;
};
