#pragma once
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <ompl/geometric/SimpleSetup.h>
namespace ob = ompl::base;

template <class T>
struct Node{
  int level;
  int id;
  T content;
  std::vector<Node<T>* > children;
};

//Tree of class Node plus Meta Information about robots and configurations
template <class T>
class Hierarchy{
  public:
    Hierarchy();

    uint NumberLevels();
    uint NumberNodesOnLevel(uint level);
    uint GetRobotIdx( uint level );
    std::vector<int> GetRobotIdxs( uint level );

    uint GetInnerRobotIdx( uint level );
    uint GetOuterRobotIdx( uint level );
    Config GetInitConfig( uint level );
    Config GetGoalConfig( uint level );

    void AddLevel( uint idx, Config qi, Config qg );
    void AddLevel( std::vector<int> idxs, Config qi, Config qg );
    void AddLevel( uint inner_idx, uint outer_idx, Config qi, Config qg );

    void AddRootNode(T content_);
    void AddNode( T content, std::vector<int> path);
    void UpdateNode( T content, std::vector<int> path);
    bool NodeExists( const std::vector<int> &path);
    bool HasChildren( const std::vector<int> &path);
    uint NumberChildren( const std::vector<int> &path);

    void DeleteNode( std::vector<int> path );
    void DeleteAllNodes();
    void DeleteAllChildNodes( std::vector<int> path );

    Node<T>* GetRootNode();
    T GetRootNodeContent();
    Node<T>* GetNode( std::vector<int> nodes );
    T GetNodeContent( std::vector<int> nodes );

    void Print();

  protected:
    //tree root
    Node<T> *root{nullptr};

    //meta information about tree
    ob::SpaceInformationPtr si;
    std::vector<int> level_robot_inner_idx;
    std::vector<std::vector<int>> level_robot_inner_idxs;
    std::vector<int> level_robot_outer_idx;
    std::vector<Config> level_q_init;
    std::vector<Config> level_q_goal;
    std::vector<int> level_number_nodes;
    void CheckLevel( uint level );
};
#include "elements/hierarchy.ipp"
