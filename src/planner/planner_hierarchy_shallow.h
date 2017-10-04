#pragma once

#include "planner/planner_hierarchy.h"

class ShallowHierarchicalMotionPlanner: public HierarchicalMotionPlanner{

  public:
    ShallowHierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_);

    virtual void Expand();
    virtual void Collapse();
    virtual void Next();
    virtual void Previous();
    virtual void DrawGL(double x_ =0.0, double y_=0.0);
    virtual void DrawGL(const GUIState&);

  private:
    std::vector<Config> solution_path;

};

