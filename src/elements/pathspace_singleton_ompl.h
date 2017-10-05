#pragma once
#include "pathspace.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceSingletonOMPL: public PathSpace{
  public:
    PathSpaceSingletonOMPL(RobotWorld *world_, PlannerInput& input_);
    virtual std::vector<PathSpace*> Decompose();
    virtual void DrawGL(const GUIState&);
    virtual bool isAtomic();
};

