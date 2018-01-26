#include "pathspace_input.h"
const CSpaceInput& PathSpaceInput::GetCSpaceInput(){
  cin = new CSpaceInput();
  cin->timestep_max = timestep_max;
  cin->timestep_min = timestep_min;
  cin->fixedBase = fixedBase;
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
PathSpaceInput* PathSpaceInput::GetNextLayer() const{
  return next_layer;
}
void PathSpaceInput::SetNextLayer(PathSpaceInput* next){
  next_layer = next;
}

std::ostream& operator<< (std::ostream& out, const PathSpaceInput& psi) 
{
  out << std::string(80, '-') << std::endl;
  out << "[PathSpaceInput]" << std::endl;
  const PathSpaceInput *next = &psi;
  while(next){
    out << " level  : " << next->level << std::endl;
    out << " type   : " << next->type << std::endl;
    out << " qinit  : " << next->q_init << std::endl;
    out << " qgoal  : " << next->q_goal << std::endl;
    cout << std::string(80, '-') << std::endl;
    next = next->GetNextLayer();
  }
  return out;
}

