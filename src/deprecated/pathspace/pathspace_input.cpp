#include "pathspace_input.h"

PathSpaceInput::PathSpaceInput()
{
  next_layer = NULL;
  exit(0);
}

PathSpaceInput::PathSpaceInput(const PlannerInput &input, int level_)
{
  level = level_;
  q_init = input.q_init;
  q_goal = input.q_goal;
  dq_init = input.dq_init;
  dq_goal = input.dq_goal;
  qMin = input.qMin;
  qMax = input.qMax;
  se3min = input.se3min;
  se3max = input.se3max;
  freeFloating = input.freeFloating;
  enableSufficiency = input.enableSufficiency;
  fixedBase = !input.freeFloating;
  kinodynamic = input.kinodynamic;
  uMin = input.uMin;
  uMax = input.uMax;

  name_sampler = input.name_sampler;
  name_algorithm = input.name_algorithm;
  epsilon_goalregion = input.epsilon_goalregion;
  max_planning_time = input.max_planning_time;
  timestep_min = input.timestep_min;
  timestep_max = input.timestep_max;

  type = input.layers.at(level).type;
  robot_idx = input.layers.at(level).inner_index;
  robot_inner_idx = input.layers.at(level).inner_index;
  robot_outer_idx = input.layers.at(level).outer_index;

  next_layer = NULL;

}

const CSpaceInput& PathSpaceInput::GetCSpaceInput()
{
  cin = new CSpaceInput();
  cin->timestep_max = timestep_max;
  cin->timestep_min = timestep_min;
  cin->fixedBase = fixedBase;
  cin->uMin = uMin;
  cin->uMax = uMax;
  cin->kinodynamic = kinodynamic;
  return *cin;
}
const StrategyInput& PathSpaceInput::GetStrategyInput()
{
  sin = new StrategyInput();
  sin->q_init = q_init;
  sin->q_goal = q_goal;
  sin->dq_init = dq_init;
  sin->dq_goal = dq_goal;
  sin->name_sampler = name_sampler;
  sin->name_algorithm = name_algorithm;
  sin->epsilon_goalregion = epsilon_goalregion;
  sin->max_planning_time = max_planning_time;
  return *sin;
}

PathSpaceInput* PathSpaceInput::GetNextLayer() const
{
  return next_layer;
}

void PathSpaceInput::SetNextLayer(PathSpaceInput* next)
{
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
    out << " robot idx         : " << next->robot_idx << std::endl;
    out << " robot idx (inner) : " << next->robot_inner_idx << std::endl;
    out << " robot idx (outer) : " << next->robot_outer_idx << std::endl;
    out << std::string(80, '-') << std::endl;
    next = next->GetNextLayer();
  }
  return out;
}

