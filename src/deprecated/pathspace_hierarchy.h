#pragma once

#include "elements/swept_volume.h"
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
namespace ob = ompl::base;

class PathNode{
  public:
    PathNode(){
      sv = NULL;
    }
    ~PathNode(){
      delete sv;
      for(uint k = 0; k < children.size(); k++){
        delete children.at(k);
      }
    }

    std::vector<Config> path;
    std::vector<ob::State*> state_path;

    int level;
    int id;

    bool isSufficient;

    std::vector<PathNode*> children;
    SweptVolume& GetSweptVolume(Robot *robot){
      if(!sv){
        sv = new SweptVolume(robot, path, 0);
      }
      return *sv;
    }
    SweptVolume *sv;

  private:
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

    uint NumberLevels();
    uint NumberNodesOnLevel(uint level);
    uint GetRobotIdx( uint level );
    uint GetInnerRobotIdx( uint level );
    uint GetOuterRobotIdx( uint level );
    Config GetInitConfig( uint level );
    Config GetGoalConfig( uint level );

    void AddLevel( uint idx, Config qi, Config qg );
    void AddLevel( uint inner_idx, uint outer_idx, Config qi, Config qg );

    void CollapsePath( std::vector<int> nodes );
    void Print();

    //methods related to a single-path hierarchical decomposition
    const std::vector<Config>& GetPathFromNodes( std::vector<int> nodes );
    void AddPath( std::vector<Config> &path_ );
    void AddPath( std::vector<Config> &path_, std::vector<int> nodes);
    PathNode* GetPathNodeFromNodes( std::vector<int> nodes );

  protected:
    //tree root
    PathNode *root;

    //meta information about tree
    ob::SpaceInformationPtr si;
    std::vector<int> level_robot_inner_idx;
    std::vector<int> level_robot_outer_idx;
    std::vector<Config> level_q_init;
    std::vector<Config> level_q_goal;
    std::vector<int> level_number_nodes;
    void CheckLevel( uint level );
};

