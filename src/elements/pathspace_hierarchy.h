#pragma once

#include "elements/swept_volume.h"
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
namespace ob = ompl::base;

struct PathNode{
  PathNode(){
    sv = NULL;
  }

  std::vector<Config> path;
  std::vector<ob::State*> state_path;

  int level;
  int node;

  std::vector<PathNode*> children;
  SweptVolume& GetSweptVolume(Robot *robot){
    if(!sv){
      sv = new SweptVolume(robot, path, 0);
    }
    return *sv;
  }
  SweptVolume *sv;
};


// hierarchical tree can be accessed by Tree( L, N),
// whereby L is the level and N is the node on the level
//
// Note: see caterpillar tree
// https://en.wikipedia.org/wiki/Caterpillar_tree
//

class PathspaceHierarchy{
  public:
    PathspaceHierarchy();

    void CreateHierarchyFromPlannerData( ob::PlannerData& pd, const ob::OptimizationObjective& obj);

    uint NumberLevels();
    uint NumberNodesOnLevel(uint level);
    uint GetRobotIdx( uint level );
    Config GetInitConfig( uint level );
    Config GetGoalConfig( uint level );
    PathNode* GetPathNodeFromNodes( std::vector<int> nodes );

    const std::vector<Config>& GetPathFromNodes( std::vector<int> nodes );

    uint AddLevel( uint ridx, Config &qi, Config &qg );
    void Print();
    void AddPath( std::vector<Config> &path_ );
    void AddPath( std::vector<Config> &path_, std::vector<int> nodes);

  protected:
    ob::SpaceInformationPtr si;

    std::vector<int> level_robot_idx;
    std::vector<Config> level_q_init;
    std::vector<Config> level_q_goal;

    std::vector<int> level_number_nodes;
    PathNode *root;

};

