#pragma once
#include "pathspace_ompl.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceMultiLevelSE2: public PathSpaceOMPL{
  public:
    PathSpaceMultiLevelSE2(RobotWorld *world_, PathSpaceInput* input_);
    std::vector<PathSpace*> Decompose() override;
};

