#include "planner/pathspace/decorator_highlighter.h"
#include "planner/pathspace/pathspace_input.h"
#include "elements/swept_volume.h"
#include "gui/drawMotionPlanner.h"

PathSpaceDecoratorHighlighter::PathSpaceDecoratorHighlighter(PathSpace* space_):
  PathSpaceDecorator(space_)
{
}

void PathSpaceDecoratorHighlighter::DrawGL(GUIState& state){
  component->DrawGL(state);

  uint ridx = component->GetPathSpaceInput()->robot_idx;
  Robot* robot = world->robots[ridx];

  //PathPiecewiseLinear* path = component->GetShortestPath();
  //if(path){
  //  const SweptVolume& sv = component->GetSweptVolume(robot);
  //  GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  //}

  const std::vector<Config> cVertices = component->GetVertices();
  const std::vector<std::pair<Config,Config>> cEdges = component->GetEdges();
  const RoadmapPtr cRoadmap = component->GetRoadmap();

  if(cVertices.size()>0){
    for(uint k = 0; k < cVertices.size(); k++){
      const Config q = cVertices.at(k);
      GLDraw::drawRobotAtConfig(robot, q, grey);
    }
  }
  if(cEdges.size()>0){
    double dmin= 0.5;
    for(uint k = 0; k < cEdges.size(); k++){
      Config q1 = cEdges.at(k).first;
      Config q2 = cEdges.at(k).second;

      double dmax = (q1-q2).norm();
      double d = 0;
      while(d < dmax){
        GLDraw::drawRobotAtConfig(robot, q1 + d*(q2-q1)/dmax, grey);
        d+=dmin;
      }
    }
  }

  if(state("draw_roadmap") && cRoadmap) cRoadmap->DrawGL(state);

}
