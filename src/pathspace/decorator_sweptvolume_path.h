#pragma once
#include "pathspace/decorator.h"
#include "elements/swept_volume.h"

class PathSpaceDecoratorSweptVolumePath: public PathSpaceDecorator{
  public:
    PathSpaceDecoratorSweptVolumePath(PathSpace* space_);
    void DrawGL(GUIState& state);
};

