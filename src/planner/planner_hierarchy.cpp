#include "planner/planner_hierarchy.h"

HierarchicalMotionPlanner::HierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_):
  MotionPlanner(world_, input_)
{
  current_level = 0;
  current_level_node = 0;
  std::cout << input << std::endl;

  if(!input.exists){
    std::cout << "No Planner Settings founds." << std::endl;
    exit(1);
  }
  std::string algorithm = input.name_algorithm;
  if(algorithm=="" || algorithm=="NONE"){
    std::cout << "No Planner Algorithm detected" << std::endl;
    exit(1);
  }

  std::vector<int> idxs = input.robot_idxs;
  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  assert(p_init.size() == p_goal.size());

  this->world->InitCollisions();


  //################################################################################
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
  }

  input.name_algorithm = algorithm.substr(13,algorithm.size()-13);

  for(uint k = 0; k < idxs.size(); k++){
    uint ridx = idxs.at(k);
    output.nested_idx.push_back(ridx);
    Robot *rk = world->robots[ridx];

    Config qi = p_init; qi.resize(rk->q.size());
    Config qg = p_goal; qg.resize(rk->q.size());

    output.nested_q_init.push_back(qi);
    output.nested_q_goal.push_back(qg);
    output.hierarchy.AddLevel( ridx, qi, qg);
  }

  //remove all nested robots except the original one
  for(uint k = 0; k < idxs.size()-1; k++){
    output.removable_robot_idxs.push_back(idxs.at(k));
  }

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Hierarchical Planner: " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << " Robots  " << std::endl;
  for(uint k = 0; k < output.nested_idx.size(); k++){
    uint ridx = output.nested_idx.at(k);
    Robot *rk = world->robots[ridx];
    std::cout << " Level" << k << "         : idx " << ridx << " name " << rk->name << std::endl;
    Config qi = output.nested_q_init.at(k);
    Config qg = output.nested_q_goal.at(k);
    std::cout << "   qinit        : " << qi << std::endl;
    std::cout << "   qgoal        : " << qi << std::endl;
  }
}

//################################################################################
bool HierarchicalMotionPlanner::solve(std::vector<int> path_idxs){

  uint Nlevel = output.hierarchy.NumberLevels();
  PathNode* node = output.hierarchy.GetPathNodeFromNodes( path_idxs );

  std::vector<Config> path_constraint = node->path;
  uint level = node->level;

  if(level >= Nlevel){
    std::cout << "reached bottom level -> nothing more to solve" << std::endl;
    return true;
  }

  std::vector<int> idxs = input.robot_idxs;

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);
  CSpaceFactory factory(input);

  //for(uint i = 0; i < idxs.size(); i++){


  uint ridx = idxs.at(level);
  Robot *ri = world->robots[ridx];
  SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,ridx,&worldsettings);
  CSpaceOMPL* cspace_i;

  if(level==0){
    cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(ri, cspace_klampt_i);
  }else{
    std::cout << "level " << level << " nyi" << std::endl;
    return true;
  }
  cspace_i->print();
  PlannerStrategyGeometric strategy;
  output.robot_idx = ridx;
  strategy.plan(input, cspace_i, output);
  return true;
  //}

}


int HierarchicalMotionPlanner::GetNumberNodesOnSelectedLevel(){
  return output.hierarchy.NumberNodesOnLevel(current_level);
}
int HierarchicalMotionPlanner::GetNumberOfLevels(){
  return output.hierarchy.NumberLevels();
}
int HierarchicalMotionPlanner::GetSelectedLevel(){
  return current_level;
}
int HierarchicalMotionPlanner::GetSelectedNode(){
  return current_level_node;
}

const std::vector<Config>& HierarchicalMotionPlanner::GetSelectedPath(){
  return output.hierarchy.GetPathFromNodes( current_path );
}
std::vector< std::vector<Config> > HierarchicalMotionPlanner::GetSiblingPaths(){

    //int current_level_node;
  std::vector< std::vector<Config> > siblings;

  std::vector<int> path = current_path;

  if(path.size()>0){
    for(uint k = 0; k < GetNumberNodesOnSelectedLevel(); k++){
      if(k==current_level_node) continue;

      path.at(path.size()-1) = k;
      const std::vector<Config>& siblingk = output.hierarchy.GetPathFromNodes( path );
      siblings.push_back(siblingk);
    }
  }
  return siblings;
}

const SweptVolume& HierarchicalMotionPlanner::GetSelectedPathSweptVolume(){
  PathNode* node = output.hierarchy.GetPathNodeFromNodes( current_path );
  uint idx = output.hierarchy.GetRobotIdx( node->level );
  Robot *robot = world->robots[idx];
  return node->GetSweptVolume(robot);
}

Robot* HierarchicalMotionPlanner::GetSelectedPathRobot(){
  uint idx = output.hierarchy.GetRobotIdx( current_level );
  Robot *robot = world->robots[idx];
  return robot;
}

const Config HierarchicalMotionPlanner::GetSelectedPathInitConfig(){
  return output.hierarchy.GetInitConfig(current_level);
}
const Config HierarchicalMotionPlanner::GetSelectedPathGoalConfig(){
  return output.hierarchy.GetGoalConfig(current_level);
}
const std::vector<int>& HierarchicalMotionPlanner::GetSelectedPathIndices(){
  return current_path;
}

int HierarchicalMotionPlanner::GetCurrentLevel(){
  return current_level;
}
//folder-like operations on hierarchical path space
void HierarchicalMotionPlanner::ExpandPath(){
  int Nmax=output.hierarchy.NumberLevels();
  if(current_level<Nmax-1){
    //current_level_node
    if(solve(current_path)){
      current_level++;
      //start from node 0 if existing
      current_path.push_back(0);
    }else{
      std::cout << "Error: Path could not be expanded" << std::endl;
    }
  }
}
void HierarchicalMotionPlanner::CollapsePath(){
  if(current_level>0){
    current_path.erase(current_path.end() - 1);
    output.hierarchy.CollapsePath( current_path );
    current_level--;
  }
}

void HierarchicalMotionPlanner::NextPath(){
  int Nmax=output.hierarchy.NumberNodesOnLevel(current_level);
  if(current_level_node<Nmax-1) current_level_node++;
  else current_level_node = 0;
  if(current_level > 0){
    current_path.at(current_level-1) = current_level_node;
  }
}
void HierarchicalMotionPlanner::PreviousPath(){
  int Nmax=output.hierarchy.NumberNodesOnLevel(current_level);
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
}
