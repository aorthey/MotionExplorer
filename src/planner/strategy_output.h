#pragma once
#include <omplapp/config.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct StrategyOutput{
  bool success;
  std::vector<Config> shortest_path;

  ob::ProblemDefinitionPtr pdef;
  ob::PlannerStatus status;
  ob::PlannerDataPtr pd;

  //deep copy
  void SetPlannerData( ob::PlannerDataPtr pd_ ){
    pd = pd_;
    pd->decoupleFromPlanner();
    /// @todo{do that only when necessary}
    pd->computeEdgeWeights();
  }

};
