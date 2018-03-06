#pragma once
#include "strategy_input.h"
#include <ompl/base/goals/GoalRegion.h>

class GoalRegionEdge: public ob::GoalRegion{
  public:
  GoalRegionEdge(const ob::SpaceInformationPtr &si):
    GoalRegion(si)
  {
  }

  virtual double distanceGoal(const ob::State *qompl) const override{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    //const ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(1);
    double d = fabs(1.0 - qomplRnSpace->values[0]);
    return d;
  }

};

struct StrategyInputEdgeGoal: public StrategyInput{
  StrategyInputEdgeGoal( const StrategyInput& in ){
    q_init = in.q_init;
    q_goal = in.q_goal;
    name_algorithm = in.name_algorithm;
    max_planning_time = in.max_planning_time;
    epsilon_goalregion = in.epsilon_goalregion;
    cspace = in.cspace;
  }
  virtual ob::GoalPtr GetGoalPtr(ob::SpaceInformationPtr si) const override{
    auto gs(std::make_shared<GoalRegionEdge>(si));
    ob::ScopedState<> goal = cspace->ConfigToOMPLState(q_goal);
    //gs->setState(goal);
    gs->setThreshold(epsilon_goalregion);
    return gs;
  }
};
