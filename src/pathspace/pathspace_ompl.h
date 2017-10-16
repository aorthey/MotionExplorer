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
    virtual std::vector<PathSpace*> Decompose() override;
    virtual void DrawGL(GUIState&) override;
    virtual bool isAtomic() const override;
};

