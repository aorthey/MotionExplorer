#pragma once

#include "planner/planner.h"
//#include "elements/pathspace_hierarchy.h"
#include "elements/hierarchy.h"
#include "ViewTree.h"
#include "elements/swept_volume.h"
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include "planner/cspace_factory.h"
#include "planner/planner_strategy_geometric.h"
#include <KrisLibrary/utils/stringutils.h>

struct SinglePathNode{
    std::vector<Config> path;
    std::vector<ob::State*> state_path;
    bool isSufficient;

    SinglePathNode(){ sv = NULL; }
    SweptVolume& GetSweptVolume(Robot *robot){
      if(!sv){
        sv = new SweptVolume(robot, path, 0);
      }
      return *sv;
    }
    SweptVolume *sv;
};

class HierarchicalMotionPlanner: public MotionPlanner{

  public:
    HierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_);

    //folder-like operations on hierarchical path space
    virtual void ExpandPath();
    virtual void CollapsePath();
    virtual void NextPath();
    virtual void PreviousPath();

    void UpdateHierarchy();

    virtual int GetNumberNodesOnSelectedLevel();
    virtual int GetNumberOfLevels();
    virtual int GetSelectedLevel();
    virtual int GetSelectedNode();
    
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
    virtual int GetCurrentLevel();

    void Print();
    virtual void DrawGL(double x_ =0.0, double y_=0.0);
    bool isActive();

    //for each level: first: number of all nodes, second: node selected on that
    //level. produces a tree of nodes with distance 1 to central path
    //const std::vector<std::pair<int,int> >& GetCaterpillarTreeIndices();

  protected:
    bool solve(std::vector<int> path_idxs);

    int current_level;
    int current_level_node;
    std::vector<int> current_path;

    Hierarchy<SinglePathNode> hierarchy;

    ViewTree viewTree;

    bool active;

};

