#include "hierarchy.h"

template <class T>
Hierarchy<T>::Hierarchy(){root=NULL;};

template <class T>
void Hierarchy<T>::CheckLevel( uint level ){
  if(level>=level_robot_inner_idx.size()){
    std::cout << "level " << level << " does not exists" << std::endl;
    exit(1);
  }
}
template <class T>
uint Hierarchy<T>::NumberLevels(){
  return level_robot_inner_idx.size();
}
template <class T>
uint Hierarchy<T>::NumberNodesOnLevel(uint level){
  CheckLevel(level);
  return level_number_nodes.at(level);
}

template <class T>
uint Hierarchy<T>::GetRobotIdx( uint level ){
  CheckLevel(level);
  return level_robot_inner_idx.at(level);
}

template <class T>
uint Hierarchy<T>::GetInnerRobotIdx( uint level ){
  CheckLevel(level);
  return GetRobotIdx(level);
}
template <class T>
uint Hierarchy<T>::GetOuterRobotIdx( uint level ){
  CheckLevel(level);
  return level_robot_outer_idx.at(level);
}

template <class T>
Config Hierarchy<T>::GetInitConfig( uint level ){
  CheckLevel(level);
  return level_q_init.at(level);
}
template <class T>
Config Hierarchy<T>::GetGoalConfig( uint level ){
  CheckLevel(level);
  return level_q_goal.at(level);
}

template <class T>
void Hierarchy<T>::AddRootNode(T content_){
  level_number_nodes.at(0)=1;
  root = new Node<T>();
  root->level = 0;
  root->id = 0;
  root->content = content_;

}
template <class T>
void Hierarchy<T>::AddLevel( uint idx, Config qi, Config qg ){
  AddLevel(idx, idx, qi, qg);
}
template <class T>
void Hierarchy<T>::AddLevel( uint inner_idx, uint outer_idx, Config qi, Config qg ){
  //if(!root){
  //  //level0
  //  level_robot_inner_idx.push_back(inner_idx);
  //  level_robot_outer_idx.push_back(outer_idx);
  //  level_q_init.push_back(qi);
  //  level_q_goal.push_back(qg);
  //  level_number_nodes.push_back(1);
  //  root = new Node<T>();
  //  root->level = 0;
  //  root->id = 0;
  //}
  level_robot_inner_idx.push_back(inner_idx);
  level_robot_outer_idx.push_back(outer_idx);
  level_q_init.push_back(qi);
  level_q_goal.push_back(qg);
  level_number_nodes.push_back(0);
}
template <class T>
void Hierarchy<T>::Print( ){
  //just the meta information
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Hierarchical Path Tree " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << " levels       : " << NumberLevels() << std::endl;
  std::cout << std::endl;
  for(uint k = 0; k < NumberLevels(); k++){
    std::cout << " level "<< k <<" nodes: " << NumberNodesOnLevel(k) << std::endl;
  }
  std::cout << "Robots ";
  for(uint k = 0; k < level_robot_inner_idx.size(); k++){
    std::cout << level_robot_inner_idx.at(k) << " ";
  }
  std::cout << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

template <class T>
Node<T>* Hierarchy<T>::GetRootNode(){
  std::vector<int> path;path.clear();
  return GetNode(path);
}
template <class T>
T Hierarchy<T>::GetRootNodeContent(){
  Node<T> *node = GetRootNode();
  return node->content;
}

template <class T>
void Hierarchy<T>::DeleteNode( std::vector<int> path ){
  Node<T>* node = GetNode( path );
  //node->children.clear();
  level_number_nodes.at(node->level+1)--;
}

template <class T>
void Hierarchy<T>::AddNode( T content_ ){
  Node<T> *next = new Node<T>();
  next->level = 1;
  next->id = root->children.size();
  next->content = content_;

  root->children.push_back(next); 
  level_number_nodes.at(next->level)++;
}

template <class T>
void Hierarchy<T>::AddNode( T content_, std::vector<int> nodes){

  Node<T> *current = GetNode(nodes);

  Node<T> *next = new Node<T>();
  next->level = current->level+1;
  next->id = current->children.size();
  next->content = content_;

  current->children.push_back(next);
  level_number_nodes.at(next->level)++;
}

template <class T>
T Hierarchy<T>::GetNodeContent( std::vector<int> path ){
  Node<T> *node = GetNode(path);
  return node->content;
}
template <class T>
Node<T>* Hierarchy<T>::GetNode( std::vector<int> path ){
  if(path.empty()) return root;

  Node<T> *current = root;

  for(uint k = 0; k < path.size(); k++){
    if(path.at(k) >= (int)current->children.size()){
      std::cout << "node " << path.at(k) << " does not exists on level " << k+1 << " in hierarchical path tree" << std::endl;
      std::cout << "input : ";
      for(uint j = 0; j < path.size(); j++){
        std::cout << path.at(j) << " ";
      }
      std::cout << std::endl;
      std::cout << "number of nodes on current level: " <<current->children.size() << std::endl;
      exit(0);
    }
    current = current->children.at( path.at(k) );
  }
  return current;
}
