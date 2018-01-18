#pragma once
#include "pathspace_ompl.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceOMPLSE2: public PathSpaceOMPL{
  public:
    PathSpaceOMPLSE2(RobotWorld *world_, PathSpaceInput* input_);
    virtual std::vector<PathSpace*> Decompose() override;
    virtual void DrawGL(GUIState&) override;
};


