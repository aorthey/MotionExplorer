#pragma once

#include "planner/planner_input.h"
#include "elements/hierarchical_roadmap.h"
#include "elements/path_pwl.h"
#include "gui/gui_state.h"
#include "gui/ViewHierarchy.h"

#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <Modeling/World.h> //RobotWorld
#include <Planning/RobotCSpace.h> //SingleRobotCSpace
#include <ompl/util/Time.h>
#include <tinyxml.h>
#include <vector>
#include <memory>

class Strategy;
typedef std::shared_ptr<Strategy> StrategyPtr;

class MotionPlanner{

  public:

    MotionPlanner(RobotWorld *world_, PlannerInput& input_);

    PlannerInput& GetInput();
    PathPiecewiseLinear* GetPath();

    //folder-like operations on hierarchical roadmap
    virtual void ExpandFull();
    virtual void ExpandSimple();
    virtual void Expand();
    virtual void Collapse();
    virtual void Next();
    virtual void Previous();

    //operations on motion planning strategy (the underlying algorithm)
    virtual void Step();
    virtual void StepOneLevel();
    virtual void AdvanceUntilSolution();
    
    virtual void DrawGL(GUIState&);
    virtual void DrawGLScreen(double x_ =0.0, double y_=0.0);

    virtual void setSelectedPath(std::vector<int> selectedPath);

    //planner will only be active if input exists and contains a valid algorithm
    bool isActive();
    void Print();
    void UpdateHierarchy();

    virtual std::string getName() const;
    virtual void Clear();

    friend std::ostream& operator<< (std::ostream& out, const MotionPlanner& planner);

    double getLastIterationTime();
  protected:
    MotionPlanner() = delete;

    //time required to execute the last action
    ompl::time::point timePointStart;
    ompl::time::point timePointEnd;
    void resetTime();
    double getTime();

    double time{0};

    //position in hierarchy
    uint current_level; //vertical level in hierarchy (tree)
    uint current_level_node; //horizontal node inside a level
    std::vector<int> current_path; //current selected path through tree

    HierarchicalRoadmapPtr hierarchy;

    RoadmapPtr Rcurrent;

    bool isHierarchical();

    bool active;
    void CreateHierarchy();

    RobotWorld *world;
    std::vector<CSpaceOMPL*> cspace_levels;
    std::vector<std::vector<CSpaceOMPL*>> cspace_stratifications;

    void InitStrategy();

    PlannerInput input;
    ViewHierarchy viewHierarchy;
    StrategyPtr strategy; //the actual algorithm implementation

    // \brief solution path of planner
    PathPiecewiseLinear *pwl; 
    CSpaceOMPL* ComputeCSpace(const std::string type, const uint robot_inner_index, const uint robot_outer_index = 0);
    CSpaceOMPL* ComputeCSpaceLayer(const Layer &layer);
};

