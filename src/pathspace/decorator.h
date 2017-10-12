#pragma once
#include "pathspace/pathspace.h"

class PathSpaceDecorator: public PathSpace{

  public:
    PathSpaceDecorator(PathSpace*);

    virtual std::vector<PathSpace*> Decompose();
    virtual bool isAtomic() const;
    virtual void DrawGL(GUIState&);

  protected:
    PathSpace* component;
};

