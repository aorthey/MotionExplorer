#pragma once
#include "pathspace.h"

//PathSpace containing a single path
//
// atomic: yes
// singlepath: yes
// 
class PathSpaceAtomic: public PathSpace{
  public:
    PathSpaceAtomic(RobotWorld *world_, PlannerInput& input_);
    virtual std::vector<PathSpace*> Decompose();
    virtual void DrawGL(const GUIState&);
    virtual bool isAtomic() const;
};
