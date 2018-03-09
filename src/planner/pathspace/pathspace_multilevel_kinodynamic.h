#pragma once
#include "pathspace_multilevel.h"

//PathSpace containing all continuous paths
//
// atomic: no
// returns singleton path
// 

class PathSpaceMultiLevelKinodynamic: public PathSpaceMultiLevel{
  public:
    PathSpaceMultiLevelKinodynamic(RobotWorld *world_, PathSpaceInput* input_);
    std::vector<PathSpace*> Decompose() override;
};

