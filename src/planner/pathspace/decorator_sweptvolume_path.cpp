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
  PathPiecewiseLinear* path = component->getShortestPathOMPL();
  //const std::vector<Config> path = component->GetShortestPath();
  if(path){
    const SweptVolume& sv = component->GetSweptVolume(robot);
    GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  }
}
