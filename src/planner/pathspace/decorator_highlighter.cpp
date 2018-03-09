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

  const RoadmapPtr cRoadmap = component->GetRoadmap();
  if(state("draw_roadmap") && cRoadmap) cRoadmap->DrawGL(state);

}
