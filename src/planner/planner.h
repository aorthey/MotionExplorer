#pragma once

#include "planner/planner_input.h"
#include "elements/hierarchy.h"
#include "elements/roadmap.h"
#include "elements/path_pwl.h"
#include "gui/gui_state.h"
#include "gui/ViewHierarchy.h"

#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <Modeling/World.h> //RobotWorld
#include <Planning/RobotCSpace.h> //SingleRobotCSpace
#include <tinyxml.h>
#include <vector>
#include <memory>

class MotionPlanner{

  public:

    MotionPlanner(RobotWorld *world_, PlannerInput& input_);

    const PlannerInput& GetInput();
    PathPiecewiseLinear* GetPath();

    //folder-like operations on hierarchical roadmap
    virtual void Expand();
    virtual void Collapse();
    virtual void Next();
    virtual void Previous();

    virtual void Step();
    virtual void Advance(double ms);
    virtual void AdvanceUntilSolution(double ms);
    
    virtual void DrawGL(GUIState&);
    virtual void DrawGLScreen(double x_ =0.0, double y_=0.0);

    //planner will only be active if input exists and contains a valid algorithm
    bool isActive();
    void Print();

    virtual std::string getName() const;

    friend std::ostream& operator<< (std::ostream& out, const MotionPlanner& planner);

  protected:
    MotionPlanner() = delete;
    void RaiseError();

    uint current_level; //vertical level in hierarchy (tree)
    uint current_level_node; //horizontal node inside a level
    std::vector<int> current_path; //current selected path through tree

    Hierarchy<RoadmapPtr> *hierarchy;
    RoadmapPtr Rcurrent;
    void UpdateHierarchy();
    bool isHierarchical();

    bool active;
    void CreateHierarchy();

    RobotWorld *world;
    PlannerInput input;
    ViewHierarchy viewHierarchy;

    PathPiecewiseLinear *pwl;
};

