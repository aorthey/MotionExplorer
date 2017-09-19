#include "path_hierarchy.h"

PathHierarchy::PathHierarchy(){root=NULL;};

uint PathHierarchy::NumberLevels(){
  return level_robot_idx.size();
}
uint PathHierarchy::NumberNodesOnLevel(uint level){
  return level_number_nodes.at(level);
}
uint PathHierarchy::GetRobotIdx( uint level ){
  return level_robot_idx.at(level);
}
Config PathHierarchy::GetInitConfig( uint level ){
  return level_q_init.at(level);
}
Config PathHierarchy::GetGoalConfig( uint level ){
  return level_q_goal.at(level);
}

PathNode* PathHierarchy::GetPathNodeFromNodes( std::vector<int> &nodes ){
  if(nodes.empty()) return root;
  PathNode *current = root;
  for(uint k = 0; k < nodes.size(); k++){
    if(nodes.at(k) >= current->children.size()){
      std::cout << "node " << nodes.at(k) << " does not exists on level " << k << " in hierarchical path tree" << std::endl;
      exit(0);
    }
    current = current->children.at( nodes.at(k) );
  }
  return current;
}

std::vector<Config> PathHierarchy::GetPath( std::vector<int> nodes ){
  PathNode *current = GetPathNodeFromNodes(nodes);
  return current->path;
}

uint PathHierarchy::AddLevel( uint ridx, Config &qi, Config &qg ){
  level_robot_idx.push_back(ridx);
  level_q_init.push_back(qi);
  level_q_goal.push_back(qg);
  level_number_nodes.push_back(0);
  if(!root){
    //level0
    root = new PathNode();
    root->path.push_back(qi);
    root->path.push_back(qg);
  }
}

void PathHierarchy::Print( ){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Hierarchical Path Tree " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << " levels       : " << NumberLevels() << std::endl;
  for(uint k = 0; k < NumberLevels(); k++){
    std::cout << " level"<< k <<" nodes: " << NumberNodesOnLevel(k) << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;

}

void PathHierarchy::AddPath( std::vector<Config> &path ){
  root->children.push_back(path); 
  level_number_nodes.at(0)++;
}

void PathHierarchy::AddPath( std::vector<Config> &path, std::vector<int> nodes){
  PathNode *current = GetPathNodeFromNodes(nodes);
  current->children.push_back(path);
  level_number_nodes.at(nodes.size())++;
}

