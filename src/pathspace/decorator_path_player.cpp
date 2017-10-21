#include "pathspace/decorator_path_player.h"
#include "pathspace/pathspace_input.h"
#include "gui/drawMotionPlanner.h"
#include "elements/path_pwl_euclid.h"

PathSpaceDecoratorPathPlayer::PathSpaceDecoratorPathPlayer(PathSpace* space_):
  PathSpaceDecorator(space_)
{

  if(component->GetShortestPath().size() > 0){
    pwl = PathPiecewiseLinearEuclidean::from_keyframes( component->GetShortestPath() );
    length = pwl->GetLength();
    t = 0;
  }
}

void PathSpaceDecoratorPathPlayer::DrawGL(GUIState& state){
  component->DrawGL(state);

  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];

  const std::vector<Config> path = component->GetShortestPath();
  if(state("draw_play_path") && path.size()>0){
    if(t<length){
      Config q = pwl->Eval(t);
      GLDraw::drawRobotAtConfig(robot, q, grey);
      t+=length/1e3;
      std::cout << t << std::endl;
    }else{
      GLDraw::drawRobotAtConfig(robot, path.back(), grey);
    }
  }

}

