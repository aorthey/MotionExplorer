#pragma once

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
    PathSpaceOnetopicCover(RobotWorld *world_, PlannerInput& input_);
    virtual std::vector<PathSpace*> Decompose();
    virtual void DrawGL(const GUIState&);
    virtual bool isAtomic() const;
};
