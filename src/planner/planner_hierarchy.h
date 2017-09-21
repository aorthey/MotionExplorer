#pragma once

#include "planner/planner.h"
#include "elements/swept_volume.h"
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

class HierarchicalMotionPlanner{

    HierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_);

    virtual bool solve();

    //folder-like operations on hierarchical path space
    void ExpandPath();
    void CollapsePath();
    void NextPath();
    void PreviousPath();

    const std::vector<Config>& GetSelectedPath();
    const SweptVolume& GetSelectedPathSweptVolume();
    const Config& GetSelectedPathInitConfig();
    const Config& GetSelectedPathGoalConfig();
    const std::vector<int>& GetSelectedPathIndices();
    const Robot* GetSelectedPathRobot();

    //for each level: first: number of all nodes, second: node selected on that
    //level. produces a tree of nodes with distance 1 to central path
    const std::vector<std::pair<int,int> >& GetCaterpillarTreeIndices();
    
  private:
    int current_level;
    int current_level_node;
    std::vector<int> current_path;

    PathspaceHierarchy hierarchy;

};

