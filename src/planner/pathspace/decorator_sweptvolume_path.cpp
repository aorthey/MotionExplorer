#include "planner/pathspace/decorator_sweptvolume_path.h"
#include "gui/drawMotionPlanner.h"

PathSpaceDecoratorSweptVolumePath::PathSpaceDecoratorSweptVolumePath(PathSpace* space_):
  PathSpaceDecorator(space_)
{
}

void PathSpaceDecoratorSweptVolumePath::DrawGL(GUIState& state){
  component->DrawGL(state);

  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const std::vector<Config> path = component->GetShortestPath();
  if(path.size()>0){
    const SweptVolume& sv = component->GetSweptVolume(robot);
    GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  }
}
