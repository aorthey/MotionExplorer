#pragma once

#include "pathspace_input.h"
#include "pathspace.h"

//PathSpaceOnetopicCover
//
// atomic: no
// singlepath: no
//
// Decomposes the space into a union of covers. Paths in one cover are
// star-shaped onetopic
// 

class PathSpaceOnetopicCover: public PathSpace{
  public:
    PathSpaceOnetopicCover(RobotWorld *world_, PathSpaceInput* input_);
    virtual std::vector<PathSpace*> Decompose();
    virtual void DrawGL(GUIState&);
    virtual bool isAtomic() const;
};
