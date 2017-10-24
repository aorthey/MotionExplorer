#pragma once
#include "pathspace_ompl.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceOMPLTwoLevel: public PathSpaceOMPL{
  public:
    PathSpaceOMPLTwoLevel(RobotWorld *world_, PathSpaceInput* input_);
    std::vector<PathSpace*> Decompose() override;
};

