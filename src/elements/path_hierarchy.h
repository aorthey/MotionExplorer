#pragma once

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>

class PathNode{
  std::vector<Config> path;

  private:
    std::vector<PathNode*> children;
}


// hierarchical tree can be accessed by Tree( L, N),
// whereby L is the level and N is the node on the level
class PathHierarchy{
  PathHierarchy();

  uint NumberLevels();
  uint NumberNodesOnLevel(uint level);
  uint GetRobotIdx( uint level );
  Config GetInitConfig( uint level );
  Config GetGoalConfig( uint level );
  PathNode* GetPathNodeFromNodes( std::vector<int> &nodes );
  std::vector<Config> GetPath( std::vector<int> nodes );
  uint AddLevel( uint ridx, Config &qi, Config &qg );
  void Print( );
  void AddPath( std::vector<Config> &path );
  void AddPath( std::vector<Config> &path, std::vector<int> nodes);

  protected:
    std::vector<int> level_robot_idx;
    std::vector<Config> level_q_init;
    std::vector<Config> level_q_goal;

    std::vector<int> level_number_nodes;
    PathNode *root;

};

