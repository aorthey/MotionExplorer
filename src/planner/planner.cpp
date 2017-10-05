

#include "planner/planner.h"
#include "planner/cspace_factory.h"
#include "planner/strategy_geometric.h"
#include "drawMotionPlanner.h"

#include "pathspace/pathspace.h"
#include "pathspace/pathspace_atomic.h"
#include "pathspace/pathspace_singleton_ompl.h"
#include "pathspace/pathspace_onetopic_cover.h"
#include "pathspace/decorator.h"
#include "pathspace/decorator_sweptvolume_path.h"
#include <KrisLibrary/utils/stringutils.h>

using namespace GLDraw;

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  active = true;
  current_level = 0;
  current_level_node = 0;
  this->world->InitCollisions();

  if(!input.exists){
    std::cout << "No Planner Settings founds." << std::endl;
    active = false;
    return;
  }

  if(!input.freeFloating){
    std::cout << " planner works only with freefloating robots right now x__X;;" << std::endl;
    exit(1);
  }

  //################################################################################
  //choose strategy
  //################################################################################
  std::string algorithm = input.name_algorithm;
  hierarchy = new Hierarchy<PathSpace*>();

  if(StartsWith(algorithm.c_str(),"hierarchical")) {
    CreateSinglePathHierarchy();
  }else if(StartsWith(algorithm.c_str(),"ompl")) {
    CreateShallowHierarchy();
  }else{
    RaiseError();
  }
}

void MotionPlanner::CreateSinglePathHierarchy(){

  std::vector<int> idxs = input.robot_idxs;
  Config p_init = input.q_init;
  Config p_goal = input.q_goal;

  for(uint k = 0; k < input.layers.size(); k++){
    int level = input.layers.at(k).level;
    int ii = input.layers.at(k).inner_index;
    int io = input.layers.at(k).outer_index;
    Robot* ri = world->robots[ii];
    Robot* ro = world->robots[io];

    Config qi = p_init; qi.resize(ri->q.size());
    Config qg = p_goal; qg.resize(ri->q.size());

    if(k==0) hierarchy->AddLevel( ii, io, qi, qg);

    hierarchy->AddLevel( ii, io, qi, qg);
  }

  //remove all nested robots except the original one
  //for(uint k = 0; k < idxs.size()-1; k++){
  //  output.removable_robot_idxs.push_back(idxs.at(k));
  //}

  std::cout << std::string(80, '-') << std::endl;
  std::cout << " Planner: " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << " Robots  " << std::endl;
  for(uint k = 0; k < hierarchy->NumberLevels(); k++){
    uint ii = hierarchy->GetInnerRobotIdx(k);
    uint io = hierarchy->GetOuterRobotIdx(k);
    Robot* ri = world->robots[ii];
    Robot* ro = world->robots[io];
    std::cout << " Level" << k << std::endl;
    std::cout << "   Robot (inner) : idx " << ii << " name " << ri->name << std::endl;
    std::cout << "   Robot (outer) : idx " << io << " name " << ro->name << std::endl;
    Config qi = hierarchy->GetInitConfig(k);
    Config qg = hierarchy->GetGoalConfig(k);
    std::cout << "      qinit      : " << qi << std::endl;
    std::cout << "      qgoal      : " << qg << std::endl;
  }

  int ii = input.layers.at(0).inner_index;
  int io = input.layers.at(0).outer_index;
  input.robot_inner_idx = ii;
  input.robot_outer_idx = io;
  std::string subalgorithm = input.name_algorithm.substr(13,input.name_algorithm.size()-13);
  input.name_algorithm = subalgorithm;
  input.robot_idx = ii;

  hierarchy->AddRootNode( new PathSpaceOnetopicCover(world, input) );
}
void MotionPlanner::CreateShallowHierarchy(){
  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  int idx = input.robot_idx;

  //  (1) space of all continuous paths
  hierarchy->AddLevel( idx, p_init, p_goal);
  //  (2) a single solution path (if it exists)
  hierarchy->AddLevel( idx, p_init, p_goal);

  //let root node by a path space, returning one path as decomposition
  hierarchy->AddRootNode( new PathSpaceSingletonOMPL(world, input) );
}

const PlannerInput& MotionPlanner::GetInput(){
  return input;
}
bool MotionPlanner::isActive(){
  return active;
}
void MotionPlanner::RaiseError(){
  std::cout << "Error Motion Planner\n" << std::endl;
  std::cout << input << std::endl;
  exit(1);
}

//bool MotionPlanner::solve(std::vector<int> path_idxs){
//  if(!active) return false;
//
//  uint Nlevel = hierarchy->NumberLevels();
//
//  Node<T>* node = hierarchy->GetNode( path_idxs );
//
//  uint level = node->level;
//
//  std::cout << "Planner level " << level << std::endl;
//  if(level >= Nlevel-1){
//    std::cout << "reached bottom level -> nothing more to solve" << std::endl;
//    return false;
//  }
//
//  WorldPlannerSettings worldsettings;
//  worldsettings.InitializeDefault(*world);
//  CSpaceFactory factory(input);
//
//  uint inner_index = hierarchy->GetInnerRobotIdx(level);
//  uint outer_index = hierarchy->GetOuterRobotIdx(level);
//  Robot* robot_inner = world->robots[inner_index];
//
//  SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
//  CSpaceOMPL* cspace_i;
//
//  std::vector<Config> path_constraint = node->content.path;
//  if(level==0){
//    cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);
//  }else if(level==1){
//    cspace_i = factory.MakeGeometricCSpacePathConstraintRollInvariance(robot_inner, cspace_klampt_i, path_constraint);
//  }else{
//    return false;
//  }
//
//  cspace_i->print();
//
//  PlannerStrategyGeometric strategy;
//  output.robot_idx = inner_index;
//  strategy.plan(input, cspace_i, output);
//
//  if(!output.success) return false;
//
//  std::vector<int> treepath;
//  if(level>0) treepath.push_back(node->id);
//
//  for(uint k = 0; k < output.paths.size(); k++){
//    T node;
//    node.path = output.paths.at(k);
//    hierarchy->AddNode( node, treepath );
//  }
//
//  return true;
//}


//std::vector< std::vector<Config> > MotionPlanner::GetSiblingPaths(){
//  std::vector< std::vector<Config> > siblings;
//
//  std::vector<int> path = current_path;
//
//  if(path.size()>0){
//    for(uint k = 0; k < hierarchy->NumberNodesOnLevel(current_level); k++){
//      if(k==current_level_node) continue;
//
//      path.at(path.size()-1) = k;
//      const std::vector<Config>& siblingk = hierarchy->GetNodeContent( path ).path;
//      siblings.push_back(siblingk);
//    }
//  }
//  return siblings;
//}

//folder-like operations on  path space
void MotionPlanner::Expand(){
  if(!active) return;

  int Nmax=hierarchy->NumberLevels();
  if(current_level<Nmax-1){
    //current_level_node

    PathSpace* P = hierarchy->GetNodeContent(current_path);
    if(P->isAtomic()){
      std::cout << "Error: Selected Path Space is Atomic." << std::endl;
    }else{
      std::vector<PathSpace*> Pvec = P->Decompose();
      if(Pvec.size()>0){
        for(uint k = 0; k < Pvec.size(); k++){
          hierarchy->AddNode( Pvec.at(k), current_path);
        }
        current_level++;
        current_level_node=0;
        current_path.push_back(current_level_node);
      }
    }
  }
  UpdateHierarchy();
}
void MotionPlanner::Collapse(){
  if(!active) return;

  if(current_level>0){
    current_path.erase(current_path.end() - 1);
    hierarchy->DeleteNode( current_path );
    current_level--;
    current_level_node = current_path.back();
  }
  UpdateHierarchy();
}

void MotionPlanner::Next(){
  if(!active) return;

  int Nmax=hierarchy->NumberNodesOnLevel(current_level);
  if(current_level_node<Nmax-1) current_level_node++;
  else current_level_node = 0;
  if(current_level > 0){
    current_path.at(current_level-1) = current_level_node;
  }
  UpdateHierarchy();
}
void MotionPlanner::Previous(){
  if(!active) return;
  int Nmax=hierarchy->NumberNodesOnLevel(current_level);
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
void MotionPlanner::UpdateHierarchy(){
  if(!active) return;

  uint L = viewTree.GetLevel();
  if(current_level == L ){
  }else{
    if(current_level < L){
      viewTree.PopLevel();
    }else{
      uint idx = hierarchy->GetRobotIdx( current_level );
      Robot *robot = world->robots[idx];
      uint N = hierarchy->NumberNodesOnLevel(current_level);
      viewTree.PushLevel(N, robot->name);
    }
  }
  viewTree.UpdateSelectionPath( current_path );
  Print();
}
void MotionPlanner::Print(){
  if(!active) return;
  hierarchy->Print();
  std::cout << "current level " << current_level << "/" << hierarchy->NumberLevels()-1 << std::endl;
  std::cout << "viewTree level " << viewTree.GetLevel() << std::endl;
  std::cout << "current node  " << current_level_node << std::endl;
  std::cout << "current path: ";
  for(uint k = 0; k < current_path.size(); k++){
    std::cout << "->" << current_path.at(k);
  }

  std::cout << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
bool MotionPlanner::isHierarchical(){
  if(!active) return false;

  uint N = hierarchy->NumberLevels();
  if(N>1) return true;
  return false;
}
void MotionPlanner::DrawGLScreen(double x_, double y_){
  if(!active) return;
  if(isHierarchical()){
    viewTree.x = x_;
    viewTree.y = y_;
    viewTree.DrawGL();
  }
}

void MotionPlanner::DrawGL(const GUIState& state){
  if(!active) return;

  uint N = hierarchy->NumberNodesOnLevel(current_level);

  PathSpace* P = new PathSpaceDecoratorSweptVolumePath( hierarchy->GetNodeContent(current_path) );
  P->DrawGL(state);
  std::cout << *P << std::endl;

  for(uint k = 0; k < N; k++){
    if(k==current_level_node) continue;
    current_path.at(current_path.size()-1) = k;
    PathSpace* Pk = hierarchy->GetNodeContent(current_path);
    Pk->DrawGL(state);
  }

  //reset
  if(current_path.size()>0){
    current_path.at(current_path.size()-1) = current_level_node;
  }

}


