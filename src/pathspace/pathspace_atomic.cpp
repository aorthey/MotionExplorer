#include "pathspace_atomic.h"
#include "drawMotionPlanner.h"

//using namespace GLDraw;
PathSpaceAtomic::PathSpaceAtomic(RobotWorld *world_, PlannerInput& input_):
  PathSpace(world_, input_)
{
}
bool PathSpaceAtomic::isAtomic(){
  return true;
}

std::vector<PathSpace*> PathSpaceAtomic::Decompose(){
  std::vector<PathSpace*> output;
  return output;
}

void PathSpaceAtomic::DrawGL(const GUIState& state){
  uint ridx = input.robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi_in = input.q_init;
  const Config qg_in = input.q_goal;

  GLColor lightGrey(0.4,0.4,0.4,0.2);
  GLColor lightGreen(0.2,0.9,0.2,0.2);
  GLColor lightRed(0.9,0.2,0.2,0.2);
  GLDraw::drawRobotAtConfig(robot, qi_in, lightGreen);
  GLDraw::drawRobotAtConfig(robot, qg_in, lightRed);

  if(vantage_path.size()>0){
    GLDraw::drawPath(vantage_path, lightGreen, 20);
    const SweptVolume& sv = GetSweptVolume(robot);
    GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  }
}

