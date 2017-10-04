#pragma once

#include "planner/planner_hierarchy.h"

class ShallowHierarchicalMotionPlanner: public HierarchicalMotionPlanner{

  public:
    ShallowHierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_);

    //folder-like operations on hierarchical path space
    virtual void ExpandPath();
    virtual void CollapsePath();
    virtual void NextPath();
    virtual void PreviousPath();

    virtual void DrawGL(double x_ =0.0, double y_=0.0);
    virtual void DrawGL(const GUIState&);

  private:
    std::vector<Config> solution_path;

};

