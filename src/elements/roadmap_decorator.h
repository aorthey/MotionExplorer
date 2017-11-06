#pragma once
#include "elements/roadmap.h"

class RoadmapDecoratorSE2: public Roadmap{
  friend Roadmap;
  public:
    RoadmapDecoratorSE2(RoadmapPtr component_);
    virtual void DrawGL(GUIState&);
    void PlotVertexIndex(uint vidx);
  private:
    RoadmapPtr component;
};

typedef std::shared_ptr<RoadmapDecoratorSE2> RoadmapDecoratorSE2Ptr;
