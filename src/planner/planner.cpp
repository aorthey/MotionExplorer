#include "planner/planner.h"
#include "planner/cspace/cspace_factory.h"
#include "gui/drawMotionPlanner.h"

#include "pathspace/pathspace_atomic.h"
#include "pathspace/pathspace_multilevel.h"
#include "pathspace/decorator.h"
#include "pathspace/decorator_highlighter.h"
#include "util.h"

using namespace GLDraw;
using namespace util;

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  current_path.clear();
  pwl = NULL;
  active = true;
  current_level = 0;
  current_level_node = 0;
  this->world->InitCollisions();

  std::string algorithm = input.name_algorithm;
  hierarchy = new Hierarchy<PathSpace*>();

  //if(StartsWith(algorithm,"hierarchical")) {
  CreateHierarchy();
  // }else if(StartsWith(algorithm,"ompl")) {
  //   CreateShallowHierarchy();
  //}else{
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "Unknown algorithm: " << algorithm << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  active = false;
  //}
}
std::string MotionPlanner::getName() const{
  return input.name_algorithm;
}

void MotionPlanner::CreateHierarchy(){

  std::vector<int> idxs = input.robot_idxs;
  //std::string subalgorithm = input.name_algorithm.substr(13,input.name_algorithm.size()-13);
  std::string algorithm = input.name_algorithm;

  PathSpaceInput *psinput_level0 = nullptr;
  PathSpaceInput *psinput = nullptr;
  for(uint k = 0; k < input.layers.size(); k++){
    int ii = input.layers.at(k).inner_index;
    int io = input.layers.at(k).outer_index;
    Robot* ri = world->robots[ii];
    Robot* ro = world->robots[io];

    if(ri==nullptr){
      std::cout << "Robot " << ii << " does not exist." << std::endl;
      exit(0);
    }
    if(ro==nullptr){
      std::cout << "Robot " << io << " does not exist." << std::endl;
      exit(0);
    }

    Config qi = input.q_init; qi.resize(ri->q.size());
    Config qg = input.q_goal; qg.resize(ri->q.size());
    Config dqi = input.dq_init; dqi.resize(ri->dq.size());
    Config dqg = input.dq_goal; dqg.resize(ri->dq.size());

    if(k==0) hierarchy->AddLevel( ii, io, qi, qg);

    hierarchy->AddLevel( ii, io, qi, qg); 

    if(k==0){
      psinput = new PathSpaceInput(input, 0);
      psinput->type = input.layers.at(0).type;

      psinput->robot_idx = ii;
      psinput->robot_inner_idx = ii;
      psinput->robot_outer_idx = io;
      psinput->name_algorithm = algorithm;

      psinput_level0 = psinput;
    }

    psinput->SetNextLayer(new PathSpaceInput(input,k));
    psinput = psinput->GetNextLayer();

    psinput->robot_idx = ii;
    psinput->robot_inner_idx = ii;
    psinput->robot_outer_idx = io;
    psinput->type = input.layers.at(k).type;

    psinput->name_algorithm = algorithm;

  }
  std::cout << *psinput_level0 << std::endl;

  //remove all nested robots except the original one
  //for(uint k = 0; k < idxs.size()-1; k++){
  //  output.removable_robot_idxs.push_back(idxs.at(k));
  //}

  if(psinput_level0!=nullptr){
    hierarchy->AddRootNode( new PathSpaceMultiLevel(world, psinput_level0) );
  }else{
    std::cout << "root node is zero" << std::endl;
    exit(0);
  }

}


/** @brief shallow hierarchy contains two pathspaces. The first path space consists of
  * the space of all continuous paths between init and goal configuration in the
  * configuration space of robot[idx]. 
  * The second path space contains of a single path if there exists a feasible
  * path or the empty set otherwise. 
  */

//void MotionPlanner::CreateShallowHierarchy(){
//
//  Config p_init = input.q_init;
//  Config p_goal = input.q_goal;
//  int idx = input.robot_idxs.back();
//  std::string type = input.layers.back().type;
//
//  //  (1) space of all continuous paths
//  hierarchy->AddLevel( idx, p_init, p_goal);
//  //  (2) a single solution path (if it exists)
//  hierarchy->AddLevel( idx, p_init, p_goal);
//
//
//  PathSpaceInput* level0 = new PathSpaceInput(input, 0);
//  level0->type = type;
//  PathSpaceInput* level1 = new PathSpaceInput(input, 1);
//  level1->type = type;
//  level0->SetNextLayer(level1);
//
//  hierarchy->AddRootNode( new PathSpaceMultiLevel(world, level0) );
//}

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

//folder-like operations on  path space
void MotionPlanner::Expand(){
  if(!active) return;

  uint Nmax=hierarchy->NumberLevels();
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

  uint Nmax=hierarchy->NumberNodesOnLevel(current_level);
  if(current_level_node<Nmax-1) current_level_node++;
  else current_level_node = 0;
  if(current_level > 0){
    current_path.at(current_level-1) = current_level_node;
  }
  UpdateHierarchy();
}
void MotionPlanner::Previous(){
  if(!active) return;
  uint Nmax=hierarchy->NumberNodesOnLevel(current_level);
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
  GetPath();
  Print();
}
void MotionPlanner::Print()
{
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

PathPiecewiseLinear* MotionPlanner::GetPath(){
  if(!active) return NULL;
  Pcurrent = hierarchy->GetNodeContent(current_path);
  pwl = Pcurrent->GetShortestPath();
  if(pwl && input.smoothPath){
    pwl->Smooth();
  }
  return pwl;
}
void MotionPlanner::DrawGL(GUIState& state){
  if(!active) return;

  uint N = hierarchy->NumberNodesOnLevel(current_level);
  //PathSpace* P = hierarchy->GetNodeContent(current_path);
  //PathSpace* P = new PathSpaceDecoratorSweptVolumePath( hierarchy->GetNodeContent(current_path) );
  //std::cout << *P << std::endl;
  Pcurrent = hierarchy->GetNodeContent(current_path);
  Pcurrent = new PathSpaceDecoratorHighlighter(Pcurrent);
  Pcurrent->DrawGL(state);

  for(uint k = 0; k < N; k++){
    if(k==current_level_node) continue;
    current_path.at(current_path.size()-1) = k;
    //PathSpace* Pk = hierarchy->GetNodeContent(current_path);
    //Pk->DrawGL(state);
  }
  if(current_path.size()>0){
    current_path.at(current_path.size()-1) = current_level_node;
  }

}
std::ostream& operator<< (std::ostream& out, const MotionPlanner& planner){
  out << std::string(80, '-') << std::endl;
  out << " Planner: " << std::endl;
  out << std::string(80, '-') << std::endl;
  out << " Robots  " << std::endl;
  for(uint k = 0; k < planner.hierarchy->NumberLevels(); k++){
    uint ii = planner.hierarchy->GetInnerRobotIdx(k);
    uint io = planner.hierarchy->GetOuterRobotIdx(k);
    Robot* ri = planner.world->robots[ii];
    Robot* ro = planner.world->robots[io];
    out << " Level" << k << std::endl;
    out << "   Robot (inner) : idx " << ii << " name " << ri->name << std::endl;
    out << "   Robot (outer) : idx " << io << " name " << ro->name << std::endl;
    Config qi = planner.hierarchy->GetInitConfig(k);
    Config qg = planner.hierarchy->GetGoalConfig(k);
    out << "      qinit      : " << qi << std::endl;
    out << "      qgoal      : " << qg << std::endl;
  }
  out << std::string(80, '-') << std::endl;
  return out;
}
