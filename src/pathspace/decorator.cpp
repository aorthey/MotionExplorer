#include "decorator.h"
#include "drawMotionPlanner.h"

PathSpaceDecorator::PathSpaceDecorator(PathSpace *component_):
  PathSpace( component_->GetWorldPtr(), component_->GetPlannerInput())
{
  component = component_;
}

bool PathSpaceDecorator::isAtomic() const{
  return component->isAtomic();
}

std::vector<PathSpace*> PathSpaceDecorator::Decompose(){
  return component->Decompose();
}

void PathSpaceDecorator::DrawGL(const GUIState& state){
  return component->DrawGL(state);
}

