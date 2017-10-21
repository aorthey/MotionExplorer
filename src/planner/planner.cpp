#include "planner/planner.h"
#include "planner/cspace/cspace_factory.h"
#include "gui/drawMotionPlanner.h"

#include "pathspace/pathspace_atomic.h"
#include "pathspace/pathspace_ompl.h"
#include "pathspace/pathspace_onetopic_cover.h"
#include "pathspace/pathspace_linear_hierarchy.h"
#include "pathspace/decorator.h"
#include "pathspace/decorator_sweptvolume_path.h"
#include "pathspace/decorator_path_player.h"
#include "pathspace/decorator_highlighter.h"
#include <KrisLibrary/utils/stringutils.h>

using namespace GLDraw;

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  active = true;
  current_level = 0;
  current_level_node = 0;
  this->world->InitCollisions();

  if(!input.freeFloating){
    std::cout << " planner works only with freefloating robots right now x__X;;" << std::endl;
    active = false;
    return;
  }

  std::string algorithm = input.name_algorithm;
  hierarchy = new Hierarchy<PathSpace*>();

  if(StartsWith(algorithm.c_str(),"hierarchical")) {
    CreateSinglePathHierarchy();
  }else if(StartsWith(algorithm.c_str(),"ompl")) {
    CreateShallowHierarchy();
  }else{
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Unknown algorithm: " << algorithm << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    active = false;
  }
}

void MotionPlanner::CreateSinglePathHierarchy(){

  std::vector<int> idxs = input.robot_idxs;
  std::string subalgorithm = input.name_algorithm.substr(13,input.name_algorithm.size()-13);

  PathSpaceInput *psinput_level0;
  PathSpaceInput *psinput;
  for(uint k = 0; k < input.layers.size(); k++){
    int level = input.layers.at(k).level;
    int ii = input.layers.at(k).inner_index;
    int io = input.layers.at(k).outer_index;
    Robot* ri = world->robots[ii];
    Robot* ro = world->robots[io];

    Config qi = input.q_init; qi.resize(ri->q.size());
    Config qg = input.q_goal; qg.resize(ri->q.size());
    Config dqi = input.dq_init; dqi.resize(ri->dq.size());
    Config dqg = input.dq_goal; dqg.resize(ri->dq.size());

    if(k==0) hierarchy->AddLevel( ii, io, qi, qg);

    hierarchy->AddLevel( ii, io, qi, qg); 

    if(k==0){
      psinput = new PathSpaceInput();
      psinput->q_init = qi;
      psinput->q_goal = qg;
      psinput->dq_init = dqi;
      psinput->dq_goal = dqg;

      psinput->robot_idx = ii;
      psinput->robot_inner_idx = ii;
      psinput->robot_outer_idx = io;
      psinput->type = input.layers.at(k).type;

      psinput->qMin = input.qMin;
      psinput->qMax = input.qMax;
      psinput->se3min = input.se3min;
      psinput->se3max = input.se3max;
      psinput->freeFloating = input.freeFloating;

      psinput->name_algorithm = subalgorithm;
      psinput->epsilon_goalregion = input.epsilon_goalregion;
      psinput->max_planning_time = input.max_planning_time;
      psinput->timestep_min = input.timestep_min;
      psinput->timestep_max = input.timestep_max;

      psinput_level0 = psinput;
    }else{
      //PathSpace *next = new PathSpaceInput();
      //psinput->SetNextLayer(new PathSpaceInput());
      //psinput = psinput->GetNextLayer();
      //psinput = new PathSpaceInput();
    }
    psinput->SetNextLayer(new PathSpaceInput());
    psinput = psinput->GetNextLayer();

    psinput->q_init = qi;
    psinput->q_goal = qg;
    psinput->dq_init = dqi;
    psinput->dq_goal = dqg;

    psinput->robot_idx = ii;
    psinput->robot_inner_idx = ii;
    psinput->robot_outer_idx = io;
    psinput->type = input.layers.at(k).type;

    psinput->qMin = input.qMin;
    psinput->qMax = input.qMax;
    psinput->se3min = input.se3min;
    psinput->se3max = input.se3max;
    psinput->freeFloating = input.freeFloating;

    psinput->name_algorithm = subalgorithm;
    psinput->epsilon_goalregion = input.epsilon_goalregion;
    psinput->max_planning_time = input.max_planning_time;
    psinput->timestep_min = input.timestep_min;
    psinput->timestep_max = input.timestep_max;
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

  //execute different flavors of hierarchical algorithms
  std::string linear = "linear";
  std::string ompl = "ompl";
  if(StartsWith(subalgorithm.c_str(),ompl.c_str())) {
    hierarchy->AddRootNode( new PathSpaceOnetopicCover(world, psinput_level0) );
    std::vector<Config> path;
    path.push_back(psinput_level0->q_init);
    path.push_back(psinput_level0->q_goal);
    hierarchy->GetRootNodeContent()->SetShortestPath( path );
  }else if(StartsWith(subalgorithm.c_str(),linear.c_str())) {
    //Zhang_2009 style + shortest path iteration
    std::string subsubalgorithm = subalgorithm.substr(linear.size()+1,subalgorithm.size()-(linear.size()+1));
    psinput_level0->name_algorithm = subsubalgorithm;
    hierarchy->AddRootNode( new PathSpaceLinearHierarchy(world, psinput_level0) );
    std::vector<Config> path;
    path.push_back(psinput_level0->q_init);
    path.push_back(psinput_level0->q_goal);
    hierarchy->GetRootNodeContent()->SetShortestPath( path );
  }else{
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Unknown algorithm: " << subalgorithm << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    active = false;
  }
}


/** @brief shallow hierarchy contains two pathspaces. The first path space consists of
  * the space of all continuous paths between init and goal configuration in the
  * configuration space of robot[idx]. 
  * The second path space contains of a single path if there exists a feasible
  * path or the empty set otherwise. 
  */

void MotionPlanner::CreateShallowHierarchy(){

  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  int idx = input.robot_idx;

  //  (1) space of all continuous paths
  hierarchy->AddLevel( idx, p_init, p_goal);
  //  (2) a single solution path (if it exists)
  hierarchy->AddLevel( idx, p_init, p_goal);

  //let root node by a path space, returning one path as decomposition
  PathSpaceInput* psinput = new PathSpaceInput();
  psinput->q_init = input.q_init;
  psinput->q_goal = input.q_goal;
  psinput->dq_init = input.dq_init;
  psinput->dq_goal = input.dq_goal;
  psinput->qMin = input.qMin;
  psinput->qMax = input.qMax;
  psinput->se3min = input.se3min;
  psinput->se3max = input.se3max;
  psinput->freeFloating = input.freeFloating;

  psinput->name_algorithm = input.name_algorithm;
  psinput->robot_idx = input.robot_idx;
  psinput->robot_inner_idx = input.robot_idx;
  psinput->robot_outer_idx = input.robot_idx;
  psinput->epsilon_goalregion = input.epsilon_goalregion;
  psinput->max_planning_time = input.max_planning_time;
  psinput->timestep_min = input.timestep_min;
  psinput->timestep_max = input.timestep_max;

  PathSpaceInput* next = new PathSpaceInput();
  next->q_init = input.q_init;
  next->q_goal = input.q_goal;
  next->dq_init = input.dq_init;
  next->dq_goal = input.dq_goal;
  next->qMin = input.qMin;
  next->qMax = input.qMax;
  next->se3min = input.se3min;
  next->se3max = input.se3max;
  next->freeFloating = input.freeFloating;
  next->robot_idx = input.robot_idx;
  next->robot_inner_idx = input.robot_idx;
  next->robot_outer_idx = input.robot_idx;
  next->SetNextLayer(NULL);

  psinput->SetNextLayer(next);

  hierarchy->AddRootNode( new PathSpaceOMPL(world, psinput) );
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
    if(current_path.size()>0){
      current_level_node = current_path.back();
    }else{
      current_level_node = 0;
    }
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

  uint L = viewHierarchy.GetLevel();
  if(current_level == L ){
  }else{
    if(current_level < L){
      viewHierarchy.PopLevel();
    }else{
      uint idx = hierarchy->GetRobotIdx( current_level );
      Robot *robot = world->robots[idx];
      uint N = hierarchy->NumberNodesOnLevel(current_level);
      viewHierarchy.PushLevel(N, robot->name);
    }
  }
  viewHierarchy.UpdateSelectionPath( current_path );
  Print();
}
void MotionPlanner::Print(){
  if(!active) return;
  hierarchy->Print();
  std::cout << "current level " << current_level << "/" << hierarchy->NumberLevels()-1 << std::endl;
  std::cout << "viewHierarchy level " << viewHierarchy.GetLevel() << std::endl;
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
    viewHierarchy.x = x_;
    viewHierarchy.y = y_;
    viewHierarchy.DrawGL();
  }
}

void MotionPlanner::DrawGL(GUIState& state){
  if(!active) return;

  uint N = hierarchy->NumberNodesOnLevel(current_level);

  //PathSpace* P = hierarchy->GetNodeContent(current_path);
  //PathSpace* P = new PathSpaceDecoratorSweptVolumePath( hierarchy->GetNodeContent(current_path) );
  //std::cout << *P << std::endl;
  Pcurrent = hierarchy->GetNodeContent(current_path);
  Pcurrent = new PathSpaceDecoratorHighlighter(Pcurrent);
  Pcurrent = new PathSpaceDecoratorPathPlayer(Pcurrent);
  Pcurrent->DrawGL(state);

  for(uint k = 0; k < N; k++){
    if(k==current_level_node) continue;
    current_path.at(current_path.size()-1) = k;
    PathSpace* Pk = hierarchy->GetNodeContent(current_path);
    Pk->DrawGL(state);
  }
  if(current_path.size()>0){
    current_path.at(current_path.size()-1) = current_level_node;
  }

}


