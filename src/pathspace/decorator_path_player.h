#pragma once
#include "pathspace/decorator.h"

class PathSpaceDecoratorPathPlayer: public PathSpaceDecorator{
  public:
    PathSpaceDecoratorPathPlayer(PathSpace* space_);
    void DrawGL(GUIState& state);
  private:
    PathPiecewiseLinearEuclidean *pwl;
    double length;
    double t;
};



