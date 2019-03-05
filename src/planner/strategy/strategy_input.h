#pragma once
#include "planner/cspace/cspace.h"
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>

namespace ob = ompl::base;

struct StrategyInput{
  Config q_init;
  Config q_goal;
  Config dq_init;
  Config dq_goal;
  std::string name_algorithm;
  std::string name_sampler;
  std::string environment_name;
  double max_planning_time;
  double epsilon_goalregion;

  //for multilevel planners
  std::vector<CSpaceOMPL*> cspace_levels;

  RobotWorld *world;

  virtual ob::GoalPtr GetGoalPtr(ob::SpaceInformationPtr si) const;

  friend std::ostream& operator<< (std::ostream&, const StrategyInput&);
};

