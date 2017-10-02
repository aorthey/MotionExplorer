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

    virtual Robot* GetOriginalRobot();
    virtual const Config GetOriginalInitConfig();
    virtual const Config GetOriginalGoalConfig();

    virtual const std::vector<Config> GetSelectedPath();
    virtual std::vector< std::vector<Config> > GetSiblingPaths();
    virtual const SweptVolume& GetSelectedPathSweptVolume();
    virtual Robot* GetSelectedPathRobot();
    virtual const Config GetSelectedPathInitConfig();
    virtual const Config GetSelectedPathGoalConfig();

    virtual const std::vector<int> GetSelectedPathIndices();
    void DrawGL(double x_ =0.0, double y_=0.0);

  private:
    std::vector<Config> solution_path;

};

