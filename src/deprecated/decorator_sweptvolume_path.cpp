#include "planner/pathspace/decorator_sweptvolume_path.h"
#include "gui/drawMotionPlanner.h"

PathSpaceDecoratorSweptVolumePath::PathSpaceDecoratorSweptVolumePath(PathSpace* space_):
  PathSpaceDecorator(space_)
{
}

void PathSpaceDecoratorSweptVolumePath::DrawGL(GUIState& state){
  component->DrawGL(state);

  //uint ridx = input->robot_idx;
  //Robot* robot = world->robots[ridx];
  //PathPiecewiseLinear* path = component->GetShortestPath();
}
