#include "pathspace_atomic.h"
#include "gui/drawMotionPlanner.h"

PathSpaceAtomic::PathSpaceAtomic(RobotWorld *world_, PathSpaceInput* input_):
  PathSpace(world_, input_)
{
}

bool PathSpaceAtomic::isAtomic() const{
  return true;
}

std::vector<PathSpace*> PathSpaceAtomic::Decompose(){
  std::vector<PathSpace*> output;
  return output;
}

void PathSpaceAtomic::DrawGL(GUIState& state){
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi_in = input->q_init;
  const Config qg_in = input->q_goal;

  if(state("planner_draw_start_configuration")) GLDraw::drawRobotAtConfig(robot, qi_in, lightGreen);
  if(state("planner_draw_goal_configuration")) GLDraw::drawRobotAtConfig(robot, qg_in, lightRed);

  if(path_ompl){
    path_ompl->DrawGL(state);
  }
}
