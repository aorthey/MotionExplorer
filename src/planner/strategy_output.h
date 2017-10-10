#pragma once
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct StrategyOutput{
  bool success;
  std::vector<Config> keyframes;
};
