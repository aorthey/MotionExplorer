#include "planner/planner.h"
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/cspace/cspace_factory.h"
#include "gui/drawMotionPlanner.h"

#include "util.h"

using namespace GLDraw;
using namespace util;

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  pwl = nullptr;
  active = true;
  current_level = 0;
  current_level_node = 0;
  current_path.clear();
  this->world->InitCollisions();
  CreateHierarchy();
}
std::string MotionPlanner::getName() const{
  return input.name_algorithm;
}

CSpaceOMPL* MotionPlanner::ComputeCSpace(const std::string type, const uint ridx)
{
  CSpaceFactory factory(input.GetCSpaceInput());

  CSpaceOMPL* cspace_level;
  if(input.freeFloating){
    if(type=="R2") {
      cspace_level = factory.MakeGeometricCSpaceRN(world, ridx, 2);
    }else if(type=="R3") {
      cspace_level = factory.MakeGeometricCSpaceRN(world, ridx, 3);
    }else if(type=="R3S2"){
      cspace_level = factory.MakeGeometricCSpaceR3S2(world, ridx);
    }else if(type=="SE3"){
      cspace_level = factory.MakeGeometricCSpaceSE3(world, ridx);
    }else if(type=="SE2"){
      cspace_level = factory.MakeGeometricCSpaceSE2(world, ridx);
    }else if(type=="SE3RN"){
      cspace_level = factory.MakeGeometricCSpace(world, ridx);
    }else{
      std::cout << "Type " << type << " not recognized" << std::endl;
      exit(0);
    }
    //if(input->enableSufficiency){
    //  cspace_level_k = new CSpaceOMPLDecoratorNecessarySufficient(cspace_level_k, input_level->robot_outer_idx);
    //}
  }else{
    if(type.substr(0,1) != "R"){
      std::cout << type.substr(0) << std::endl;
      std::cout << "fixed robots needs to have configuration space RN, but has " << type << std::endl;
      exit(0);
    }

    std::string str_dimension = type.substr(1);
    int N = boost::lexical_cast<int>(str_dimension);
    cspace_level = factory.MakeGeometricCSpaceFixedBase(world, ridx, N);

  }
  return cspace_level;
}
void MotionPlanner::CreateHierarchy(){
  hierarchy = std::make_shared<HierarchicalRoadmap>();
  std::vector<int> idxs = input.robot_idxs;
  std::string algorithm = input.name_algorithm;

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  //#########################################################################
  //For each level, compute the inputs/cspaces for each robot
  //#########################################################################
  if(util::StartsWith(algorithm, "hierarchy")){
    for(uint k = 0; k < input.layers.size(); k++){
      //#########################################################################
      //LEVEL k: get robots and compute init/goal
      //#########################################################################
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

      //#########################################################################
      //LEVEL k: given robot k, compute its cspace (free float vs fixed base)
      //#########################################################################
      std::string type = input.layers.at(k).type;
      CSpaceOMPL *cspace_level_k = ComputeCSpace(type, ii);
      cspace_levels.push_back( cspace_level_k );

      if(k==0){
        //add a root node
        hierarchy->AddLevel( ii, io, qi, qg);
      }
      hierarchy->AddLevel( ii, io, qi, qg); 
    }

    //empty roadmap on the highest level (so that we can collapse the whole
    //hierarchy into a singleton node)
    hierarchy->AddRootNode( std::make_shared<Roadmap>() ); 
    std::vector<int> path;
    for(uint k = 0; k < cspace_levels.size(); k++){
      hierarchy->AddNode( std::make_shared<Roadmap>(), path ); 
      path.push_back(0);
    }

    hierarchy->Print();
  }else{
    //shallow algorithm

    Config qi = input.q_init;
    Config qg = input.q_goal;
    Config dqi = input.dq_init;
    Config dqg = input.dq_goal;
    uint ridx = input.robot_idx;

    std::string type = input.layers.back().type;
    CSpaceOMPL *cspace_level = ComputeCSpace(type, ridx);
    cspace_levels.push_back( cspace_level );

    //two levels, so we can collapse the roadmap 
    hierarchy->AddLevel( ridx, ridx, qi, qg);
    hierarchy->AddLevel( ridx, ridx, qi, qg);

    hierarchy->AddRootNode( std::make_shared<Roadmap>() ); 
    std::vector<int> path;
    hierarchy->AddNode( std::make_shared<Roadmap>(), path ); 
  }

}

void MotionPlanner::Clear()
{
}

void MotionPlanner::Step()
{
  std::cout << "NIY" << std::endl;
}
void MotionPlanner::Advance(double ms)
{
  std::cout << "NIY" << std::endl;
}
void MotionPlanner::AdvanceUntilSolution()
{
  StrategyOutput output(cspace_levels.back());

  StrategyInput strategy_input = input.GetStrategyInput();
  strategy_input.cspace_levels = cspace_levels;
  strategy_input.cspace = cspace_levels.back();
  strategy_input.world = world;

  strategy.plan(strategy_input, output);

  std::cout << output << std::endl;

  output.GetHierarchicalRoadmap( hierarchy, cspace_levels );
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

//folder-like operations on hierarchical roadmap
void MotionPlanner::Expand(){
  if(!active) return;

  uint Nmax=hierarchy->NumberLevels();
  if(current_level<Nmax-1){
    current_level++;
    current_level_node=0;
    current_path.push_back(current_level_node);
  }
  UpdateHierarchy();
}

void MotionPlanner::Collapse(){
  if(!active) return;

  if(current_level>0){
    current_path.erase(current_path.end() - 1);
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
  Rcurrent = hierarchy->GetNodeContent(current_path);
  pwl = Rcurrent->GetShortestPath();
  if(pwl && input.smoothPath){
    pwl->Smooth();
  }
  return pwl;
}
void MotionPlanner::DrawGL(GUIState& state){
  if(!active) return;

  //uint N = hierarchy->NumberNodesOnLevel(current_level);
  Rcurrent = hierarchy->GetNodeContent(current_path);
  Rcurrent->DrawGL(state);

  uint ridx = hierarchy->GetRobotIdx(current_level);
  Robot* robot = world->robots[ridx];
  const Config qi = hierarchy->GetInitConfig(current_level);
  const Config qg = hierarchy->GetGoalConfig(current_level);
  if(state("planner_draw_start_configuration")) GLDraw::drawRobotAtConfig(robot, qi, green);
  if(state("planner_draw_goal_configuration")) GLDraw::drawRobotAtConfig(robot, qg, red);

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
