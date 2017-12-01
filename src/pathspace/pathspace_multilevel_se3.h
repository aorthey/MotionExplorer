#pragma once
#include "pathspace_ompl.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceMultiLevelSE3: public PathSpaceOMPL{
  public:
    PathSpaceMultiLevelSE3(RobotWorld *world_, PathSpaceInput* input_);
    std::vector<PathSpace*> Decompose() override;
};

