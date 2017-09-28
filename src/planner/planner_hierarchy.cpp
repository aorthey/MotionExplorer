#include "planner/planner_hierarchy.h"

HierarchicalMotionPlanner::HierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_):
  MotionPlanner(world_, input_)
{
  active = true;
  current_level = 0;
  current_level_node = 0;
  std::cout << input << std::endl;

  if(!input.exists){
    std::cout << "No Planner Settings founds." << std::endl;
    active = false;
    return;
    //exit(1);
  }
  std::string algorithm = input.name_algorithm;
  if(algorithm=="" || algorithm=="NONE"){
    std::cout << "No Planner Algorithm detected" << std::endl;
    active = false;
    return;
    //exit(1);
  }

  std::vector<int> idxs = input.robot_idxs;
  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  assert(p_init.size() == p_goal.size());

  this->world->InitCollisions();

  //################################################################################
  std::cout << algorithm << std::endl;
  if(StartsWith(algorithm.c_str(),"hierarchical")) {
    if(algorithm.size()<14){
      std::cout << "Error Hierarchical Algorithm: \n             Usage hierarchical:<algorithm>  \n            Example: hierarchical:ompl:rrt" << std::endl;
      std::cout << "your input: " << algorithm.c_str() << std::endl;
      exit(1);
    }
    if(!input.freeFloating){
      std::cout << "Hierarchical planner works only with freefloating robots right now x__X;;" << std::endl;
      exit(1);
    }
  }else{
    return;
  }

  input.name_algorithm = algorithm.substr(13,algorithm.size()-13);
  //################################################################################

  for(uint k = 0; k < input.layers.size(); k++){
    int level = input.layers.at(k).level;
    int ii = input.layers.at(k).inner_index;
    int io = input.layers.at(k).outer_index;
    Robot *ri = world->robots[ii];
    Robot *ro = world->robots[io];

    Config qi = p_init; qi.resize(ri->q.size());
    Config qg = p_goal; qg.resize(ri->q.size());

    hierarchy.AddLevel( ii, io, qi, qg);
  }
  //uint ridx = idxs.at(0);
  //Robot *r0 = world->robots[ridx];

  //remove all nested robots except the original one
  for(uint k = 0; k < idxs.size()-1; k++){
    output.removable_robot_idxs.push_back(idxs.at(k));
  }

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Hierarchical Planner: " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << " Robots  " << std::endl;
  for(uint k = 0; k < input.layers.size(); k++){
    uint ii = hierarchy.GetInnerRobotIdx(k);
    uint io = hierarchy.GetOuterRobotIdx(k);
    Robot *ri = world->robots[ii];
    Robot *ro = world->robots[io];
    std::cout << " Level" << k << std::endl;
    std::cout << "   Robot (inner) : idx " << ii << " name " << ri->name << std::endl;
    std::cout << "   Robot (outer) : idx " << io << " name " << ro->name << std::endl;
    Config qi = hierarchy.GetInitConfig(k);
    Config qg = hierarchy.GetGoalConfig(k);
    std::cout << "      qinit      : " << qi << std::endl;
    std::cout << "      qgoal      : " << qg << std::endl;
  }
}

Robot* HierarchicalMotionPlanner::GetOriginalRobot(){
  uint N = hierarchy.NumberLevels()-1;
  uint ridx = hierarchy.GetRobotIdx(N);
  return world->robots[ridx];
}
const Config HierarchicalMotionPlanner::GetOriginalInitConfig(){
  uint N = hierarchy.NumberLevels()-1;
  return hierarchy.GetInitConfig(N);
}
const Config HierarchicalMotionPlanner::GetOriginalGoalConfig(){
  uint N = hierarchy.NumberLevels()-1;
  return hierarchy.GetGoalConfig(N);
}
//################################################################################
bool HierarchicalMotionPlanner::solve(std::vector<int> path_idxs){
  if(!active) return false;

  uint Nlevel = hierarchy.NumberLevels();
  PathNode* node = hierarchy.GetPathNodeFromNodes( path_idxs );

  uint level = node->level;

  std::cout << "Planner level " << level << std::endl;
  if(level >= Nlevel-1){
    std::cout << "reached bottom level -> nothing more to solve" << std::endl;
    return false;
  }

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);
  CSpaceFactory factory(input);

  uint inner_index = hierarchy.GetInnerRobotIdx(level);
  uint outer_index = hierarchy.GetOuterRobotIdx(level);
  Robot *robot_inner = world->robots[inner_index];

  SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
  CSpaceOMPL* cspace_i;

  std::vector<Config> path_constraint = node->path;
  if(level==0){
    cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);
  }else if(level==1){
    cspace_i = factory.MakeGeometricCSpacePathConstraintRollInvariance(robot_inner, cspace_klampt_i, path_constraint);
  }else{
    return true;
  }
  cspace_i->print();
  PlannerStrategyGeometric strategy;
  output.robot_idx = inner_index;
  strategy.plan(input, cspace_i, output);

  if(!output.success) return false;

  std::vector<int> nodes;
  if(level>0) nodes.push_back(node->id);

  for(uint k = 0; k < output.paths.size(); k++){
    hierarchy.AddPath( output.paths.at(k), nodes );
  }

  return true;
}


int HierarchicalMotionPlanner::GetNumberNodesOnSelectedLevel(){
  return hierarchy.NumberNodesOnLevel(current_level);
}
int HierarchicalMotionPlanner::GetNumberOfLevels(){
  return hierarchy.NumberLevels();
}
int HierarchicalMotionPlanner::GetSelectedLevel(){
  return current_level;
}
int HierarchicalMotionPlanner::GetSelectedNode(){
  return current_level_node;
}

const std::vector<Config>& HierarchicalMotionPlanner::GetSelectedPath(){
  return hierarchy.GetPathFromNodes( current_path );
}
std::vector< std::vector<Config> > HierarchicalMotionPlanner::GetSiblingPaths(){

    //int current_level_node;
  std::vector< std::vector<Config> > siblings;

  std::vector<int> path = current_path;

  if(path.size()>0){
    for(uint k = 0; k < GetNumberNodesOnSelectedLevel(); k++){
      if(k==current_level_node) continue;

      path.at(path.size()-1) = k;
      const std::vector<Config>& siblingk = hierarchy.GetPathFromNodes( path );
      siblings.push_back(siblingk);
    }
  }
  return siblings;
}

const SweptVolume& HierarchicalMotionPlanner::GetSelectedPathSweptVolume(){
  PathNode* node = hierarchy.GetPathNodeFromNodes( current_path );
  uint idx = hierarchy.GetRobotIdx( node->level );
  Robot *robot = world->robots[idx];
  return node->GetSweptVolume(robot);
}

Robot* HierarchicalMotionPlanner::GetSelectedPathRobot(){
  uint idx = hierarchy.GetRobotIdx( current_level );
  Robot *robot = world->robots[idx];
  return robot;
}

const Config HierarchicalMotionPlanner::GetSelectedPathInitConfig(){
  return hierarchy.GetInitConfig(current_level);
}
const Config HierarchicalMotionPlanner::GetSelectedPathGoalConfig(){
  return hierarchy.GetGoalConfig(current_level);
}
const std::vector<int>& HierarchicalMotionPlanner::GetSelectedPathIndices(){
  return current_path;
}

int HierarchicalMotionPlanner::GetCurrentLevel(){
  return current_level;
}

//folder-like operations on hierarchical path space
void HierarchicalMotionPlanner::ExpandPath(){
  if(!active) return;
  int Nmax=hierarchy.NumberLevels();
  if(current_level<Nmax-1){
    //current_level_node
    if(solve(current_path)){
      //start from node 0 if existing
      current_level++;
      current_level_node=0;
      current_path.push_back(current_level_node);
    }else{
      std::cout << "Error: Path could not be expanded" << std::endl;
    }
  }
  UpdateHierarchy();
}
void HierarchicalMotionPlanner::CollapsePath(){
  if(!active) return;
  if(current_level>0){
    current_path.erase(current_path.end() - 1);
    hierarchy.CollapsePath( current_path );
    current_level--;
    current_level_node = current_path.back();
  }
  UpdateHierarchy();
}

void HierarchicalMotionPlanner::NextPath(){
  if(!active) return;
  int Nmax=hierarchy.NumberNodesOnLevel(current_level);
  if(current_level_node<Nmax-1) current_level_node++;
  else current_level_node = 0;
  if(current_level > 0){
    current_path.at(current_level-1) = current_level_node;
  }
  UpdateHierarchy();
}
void HierarchicalMotionPlanner::PreviousPath(){
  if(!active) return;
  int Nmax=hierarchy.NumberNodesOnLevel(current_level);
  if(current_level_node>0) current_level_node--;
  else{
    if(Nmax>0){
      current_level_node = Nmax-1;
    }else{
      current_level_node = 0;
    }
  }
  if(current_level > 0){
    current_path.at(current_level-1) = current_level_node;
  }
  UpdateHierarchy();
}
void HierarchicalMotionPlanner::UpdateHierarchy(){
  uint L = viewTree.GetLevel();
  if(current_level == L ){
  }else{
    if(current_level < L){
      viewTree.PopLevel();
    }else{
      uint idx = hierarchy.GetRobotIdx( current_level );
      Robot *robot = world->robots[idx];
      uint N = GetNumberNodesOnSelectedLevel();
      viewTree.PushLevel(N, robot->name);
    }
  }
  viewTree.UpdateSelectionPath( current_path );
  Print();
}
void HierarchicalMotionPlanner::Print(){
  if(!active) return;
  hierarchy.Print();
  std::cout << "current level " << current_level << "/" << hierarchy.NumberLevels()-1 << std::endl;
  std::cout << "viewTree level " << viewTree.GetLevel() << std::endl;
  std::cout << "current node  " << current_level_node << std::endl;
  std::cout << "current path: ";
  for(uint k = 0; k < current_path.size(); k++){
    std::cout << "->" << current_path.at(k);
  }

  std::cout << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
void HierarchicalMotionPlanner::DrawGL(double x_, double y_){
  if(!active) return;
  viewTree.x = x_;
  viewTree.y = y_;
  viewTree.DrawGL();
}
bool HierarchicalMotionPlanner::isActive(){
  return active;
}
