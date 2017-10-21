#include "planner/strategy/strategy_input.h"

ob::GoalPtr StrategyInput::GetGoalPtr(ob::SpaceInformationPtr si) const{
  auto gs(std::make_shared<ob::GoalState>(si));
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(q_goal);
  gs->setState(goal);
  gs->setThreshold(epsilon_goalregion);
  return gs;
}

std::ostream& operator<< (std::ostream& out, const StrategyInput& si) 
{
  out << std::string(80, '-') << std::endl;
  out << "[StrategyInput]" << std::endl;
  out << si.q_init << std::endl;
  out << si.q_goal << std::endl;
  out << si.name_algorithm << std::endl;
  cout << std::string(80, '-') << std::endl;
  return out;
}
