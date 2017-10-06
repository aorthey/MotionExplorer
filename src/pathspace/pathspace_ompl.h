#pragma once
#include "pathspace.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceOMPL: public PathSpace{
  public:
    PathSpaceOMPL(RobotWorld *world_, PlannerInput& input_);
    virtual std::vector<PathSpace*> Decompose();
    virtual void DrawGL(const GUIState&);
    virtual bool isAtomic() const;
};

