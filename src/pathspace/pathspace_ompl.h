#pragma once
#include "pathspace_input.h"
#include "pathspace.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceOMPL: public PathSpace{
  public:
    PathSpaceOMPL(RobotWorld *world_, PathSpaceInput* input_);
    virtual std::vector<PathSpace*> Decompose();
    virtual void DrawGL(GUIState&);
    virtual bool isAtomic() const;
};

