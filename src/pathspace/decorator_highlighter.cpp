#include "pathspace/decorator_highlighter.h"
#include "drawMotionPlanner.h"

PathSpaceDecoratorHighlighter::PathSpaceDecoratorHighlighter(PathSpace* space_):
  PathSpaceDecorator(space_)
{
}

void PathSpaceDecoratorHighlighter::DrawGL(GUIState& state){
  component->DrawGL(state);

  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];

  GLColor grey(0.8,0.8,0.8,0.5);
  const std::vector<Config> path = component->GetShortestPath();
  if(path.size()>0){
    const SweptVolume& sv = component->GetSweptVolume(robot);
    GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  }
  if(vertices.size()>0){
    for(uint k = 0; k < vertices.size(); k++){
      const Config q = vertices.at(k);
      GLDraw::drawRobotAtConfig(robot, q, grey);
    }
  }
  if(edges.size()>0){
    double dmin= 0.5;
    for(uint k = 0; k < edges.size(); k++){
      Config q1 = edges.at(k).first;
      Config q2 = edges.at(k).second;

      double dmax = (q1-q2).norm();
      double d = 0;
      while(d < dmax){
        GLDraw::drawRobotAtConfig(robot, q1 + d*(q2-q1)/dmax, grey);
        d+=dmin;
      }
    }
  }

  GLColor magenta(0.9,0.1,0.9,0.5);
  if(paths.size()>0){
    for(uint k = 0; k < paths.size(); k++){
      const std::vector<Config> p = paths.at(k);
      for(uint j = 0; j < p.size(); j++){
        GLDraw::drawPath( p, magenta, 3, 1);
      }
    }
  }

}
