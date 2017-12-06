#include "pathspace_input.h"
const CSpaceInput& PathSpaceInput::GetCSpaceInput(){
  cin = new CSpaceInput();
  cin->timestep_max = timestep_max;
  cin->timestep_min = timestep_min;
  return *cin;
}
const StrategyInput& PathSpaceInput::GetStrategyInput(){
  sin = new StrategyInput();
  sin->q_init = q_init;
  sin->q_goal = q_goal;
  sin->name_sampler = name_sampler;
  sin->name_algorithm = name_algorithm;
  sin->epsilon_goalregion = epsilon_goalregion;
  sin->max_planning_time = max_planning_time;
  return *sin;
}
PathSpaceInput* PathSpaceInput::GetNextLayer(){
  return next_layer;
}
void PathSpaceInput::SetNextLayer(PathSpaceInput* next){
  next_layer = next;
}

std::ostream& operator<< (std::ostream& out, const PathSpaceInput& psi) 
{
  out << std::string(80, '-') << std::endl;
  out << "[PathSpaceInput]" << std::endl;
  out << " level  : " << psi.level << std::endl;
  out << " type   : " << psi.type << std::endl;
  out << " qinit  : " << psi.q_init << std::endl;
  out << " qgoal  : " << psi.q_goal << std::endl;
  cout << std::string(80, '-') << std::endl;
  return out;
}

