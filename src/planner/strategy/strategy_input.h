#pragma once
#include "planner/cspace/cspace.h"
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct StrategyInput{
  Config q_init;
  Config q_goal;
  std::string name_algorithm;
  double max_planning_time;
  double epsilon_goalregion;
  CSpaceOMPL *cspace;
  CSpaceOMPL *cspace_level1;
  RobotWorld *world;

  virtual ob::GoalPtr GetGoalPtr(ob::SpaceInformationPtr si) const;

  friend std::ostream& operator<< (std::ostream&, const StrategyInput&);
};

