#pragma once
#include "pathspace.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceMultiLevel: public PathSpace{
  public:
    PathSpaceMultiLevel(RobotWorld *world_, PathSpaceInput* input_);
    std::vector<PathSpace*> Decompose() override;
    virtual void DrawGL(GUIState&) override;
    virtual bool isAtomic() const override;
};

