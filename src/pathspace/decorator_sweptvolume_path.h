#pragma once
#include "pathspace/decorator.h"

class PathSpaceDecoratorSweptVolumePath: public PathSpaceDecorator{
  public:
    PathSpaceDecoratorSweptVolumePath(PathSpace* space_);
    void DrawGL(GUIState& state);
};

