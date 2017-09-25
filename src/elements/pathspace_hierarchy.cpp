#include "elements/pathspace_hierarchy.h"
#include "algorithms/onetopic_reduction.h"

PathspaceHierarchy::PathspaceHierarchy(){root=NULL;};

uint PathspaceHierarchy::NumberLevels(){
  return level_robot_idx.size();
}
uint PathspaceHierarchy::NumberNodesOnLevel(uint level){
  return level_number_nodes.at(level);
}
uint PathspaceHierarchy::GetRobotIdx( uint level ){
  if(level>=level_robot_idx.size()){
    std::cout << "level " << level << " does not exists" << std::endl;
    exit(1);
  }
  return level_robot_idx.at(level);
}
Config PathspaceHierarchy::GetInitConfig( uint level ){
  return level_q_init.at(level);
}
Config PathspaceHierarchy::GetGoalConfig( uint level ){
  return level_q_goal.at(level);
}

PathNode* PathspaceHierarchy::GetPathNodeFromNodes( std::vector<int> nodes ){
  if(nodes.empty()) return root;

  PathNode *current = root;

  for(uint k = 0; k < nodes.size(); k++){
    //std::cout << "children: " << current->children.size() << std::endl;
    if(nodes.at(k) >= current->children.size()){
      std::cout << "node " << nodes.at(k) << " does not exists on level " << k+1 << " in hierarchical path tree" << std::endl;
      std::cout << "input : ";
      for(uint j = 0; j < nodes.size(); j++){
        std::cout << nodes.at(j) << " ";
      }
      std::cout << std::endl;
      //return current;
      exit(0);
    }
    current = current->children.at( nodes.at(k) );
  }
  return current;
}

const std::vector<Config>& PathspaceHierarchy::GetPathFromNodes( std::vector<int> nodes ){
  PathNode* node = GetPathNodeFromNodes( nodes );
  return node->path;
}

uint PathspaceHierarchy::AddLevel( uint ridx, Config &qi, Config &qg ){
  if(!root){
    //level0
    level_robot_idx.push_back(ridx);
    level_q_init.push_back(qi);
    level_q_goal.push_back(qg);
    level_number_nodes.push_back(1);
    root = new PathNode();
    root->level = 0;
    root->id = 0;
    root->path.clear();
    root->path.push_back(qi);
    root->path.push_back(qg);
  }
  level_robot_idx.push_back(ridx);
  level_q_init.push_back(qi);
  level_q_goal.push_back(qg);
  level_number_nodes.push_back(0);
}

void PathspaceHierarchy::Print( ){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Hierarchical Path Tree " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << " levels       : " << NumberLevels() << std::endl;
  std::cout << std::endl;
  for(uint k = 0; k < NumberLevels(); k++){
    std::cout << " level "<< k <<" nodes: " << NumberNodesOnLevel(k) << std::endl;
  }
  std::cout << "Robots ";
  for(uint k = 0; k < level_robot_idx.size(); k++){
    std::cout << level_robot_idx.at(k) << " ";
  }
  std::cout << std::endl;
  std::cout << std::string(80, '-') << std::endl;


}

void PathspaceHierarchy::CollapsePath( std::vector<int> nodes ){
  PathNode* collapsed_node = GetPathNodeFromNodes( nodes );
  collapsed_node->children.clear();
  level_number_nodes.at(collapsed_node->level+1)=0;
}

void PathspaceHierarchy::AddPath( std::vector<Config> &path_ ){
  PathNode* newpath = new PathNode();
  newpath->level = 1;
  newpath->id = root->children.size();
  newpath->path = path_;
  root->children.push_back(newpath); 
  level_number_nodes.at(newpath->level)++;
}

void PathspaceHierarchy::AddPath( std::vector<Config> &path_, std::vector<int> nodes){

  PathNode *current = GetPathNodeFromNodes(nodes);
  
  PathNode *newpath = new PathNode();
  newpath->level = current->level+1;
  newpath->id = current->children.size();
  newpath->path = path_;

  current->children.push_back(newpath);
  level_number_nodes.at(newpath->level)++;
}

