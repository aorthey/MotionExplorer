#pragma once

#include "planner/planner.h"
#include "ViewTree.h"
#include "elements/swept_volume.h"
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include "planner/cspace_factory.h"
#include "planner/planner_strategy_geometric.h"
#include <KrisLibrary/utils/stringutils.h>

class HierarchicalMotionPlanner: public MotionPlanner{

  public:
    HierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_);

    bool solve(std::vector<int> path_idxs);

    //folder-like operations on hierarchical path space
    void ExpandPath();
    void CollapsePath();
    void NextPath();
    void PreviousPath();
    void UpdateHierarchy();

    int GetNumberNodesOnSelectedLevel();
    int GetNumberOfLevels();
    int GetSelectedLevel();
    int GetSelectedNode();

    const std::vector<Config>& GetSelectedPath();

    std::vector< std::vector<Config> > GetSiblingPaths();

    const SweptVolume& GetSelectedPathSweptVolume();
    Robot* GetSelectedPathRobot();

    const Config GetSelectedPathInitConfig();
    const Config GetSelectedPathGoalConfig();
    const std::vector<int>& GetSelectedPathIndices();
    int GetCurrentLevel();

    void Print();
    void DrawGL(double x_ =0.0, double y_=0.0);
    bool isActive();

    //for each level: first: number of all nodes, second: node selected on that
    //level. produces a tree of nodes with distance 1 to central path
    //const std::vector<std::pair<int,int> >& GetCaterpillarTreeIndices();
    
    Robot* GetOriginalRobot();
    const Config GetOriginalInitConfig();
    const Config GetOriginalGoalConfig();
  private:
    int current_level;
    int current_level_node;
    std::vector<int> current_path;

    PathspaceHierarchy hierarchy;

    ViewTree viewTree;

    bool active;

};

