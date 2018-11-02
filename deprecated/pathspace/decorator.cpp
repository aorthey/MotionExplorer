#include "decorator.h"
#include "gui/drawMotionPlanner.h"

PathSpaceDecorator::PathSpaceDecorator(PathSpace *component_):
  PathSpace( component_->GetWorldPtr(), component_->GetPathSpaceInput())
{
  component = component_;
}

bool PathSpaceDecorator::isAtomic() const{
  return component->isAtomic();
}

std::vector<PathSpace*> PathSpaceDecorator::Decompose(){
  return component->Decompose();
}

void PathSpaceDecorator::DrawGL(GUIState& state){
  return component->DrawGL(state);
}

