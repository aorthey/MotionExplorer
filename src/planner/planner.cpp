#include "planner/planner.h"
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_kinodynamic.h"
#include "planner/cspace/cspace_factory.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "gui/drawMotionPlanner.h"
#include "util.h"
#include <ompl/geometric/planners/quotientspace/Explorer.h>

#include <boost/lexical_cast.hpp>

using namespace GLDraw;

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  pwl = nullptr;
  active = true;
  current_level = 0;
  current_level_node = 0;
  current_path.clear();
  this->world->InitCollisions();
  if(input.kinodynamic){
    strategy = std::make_shared<StrategyKinodynamicMultiLevel>();
  }else{
    strategy = std::make_shared<StrategyGeometricMultiLevel>();
  }
  timePointStart = ompl::time::now();

  CreateHierarchy();
}
std::string MotionPlanner::getName() const{
  return input.name_algorithm;
}

CSpaceOMPL* MotionPlanner::ComputeMultiAgentCSpace(const Layer &layer){
  CSpaceFactory factory(input.GetCSpaceInput());
  std::vector<CSpaceOMPL*> cspace_levels;

  for(uint k = 0; k < layer.ids.size(); k++){
    int rk = layer.ids.at(k);
    if(rk >= (int)world->robots.size()){
      OMPL_ERROR("Robot Index ID %d does not exists. (Check XML)", rk);
      throw "wrong ID";
    }
    std::string type = layer.types.at(k);
    bool freeFloating = layer.freeFloating.at(k);
    // std::cout << type << " " << (freeFloating?"FREEFLOATING":"FIXED") << std::endl;
    CSpaceOMPL* cspace_level_k = ComputeCSpace(type, rk, freeFloating);
    cspace_levels.push_back(cspace_level_k);
  }
  CSpaceOMPLMultiAgent* cspace_multiagent = factory.MakeGeometricCSpaceMultiAgent(cspace_levels);
  cspace_multiagent->setNextLevelRobotPointers(layer.ptr_to_next_level_ids);
  return cspace_multiagent;
}

CSpaceOMPL* MotionPlanner::ComputeCSpace(const std::string type, const uint robot_idx, bool freeFloating)
{
  CSpaceFactory factory(input.GetCSpaceInput());

  CSpaceOMPL* cspace_level;
  if(type=="EMPTY_SET") 
  {
    cspace_level = factory.MakeEmptySetSpace(world);
  }else{
    if(freeFloating)
    {
      if(type=="R2") {
        cspace_level = factory.MakeGeometricCSpaceRN(world, robot_idx, 2);
      }else if(type=="R3") {
        cspace_level = factory.MakeGeometricCSpaceRN(world, robot_idx, 3);
      }else if(type=="R3S2"){
        cspace_level = factory.MakeGeometricCSpaceR3S2(world, robot_idx);
      }else if(type=="SE3"){
        cspace_level = factory.MakeGeometricCSpaceSE3(world, robot_idx);
      }else if(type=="SE2"){
        cspace_level = factory.MakeGeometricCSpaceSE2(world, robot_idx);
      }else if(type=="SE2RN"){
        cspace_level = factory.MakeGeometricCSpaceSE2RN(world, robot_idx);
      }else if(type=="SE3RN"){
        cspace_level = factory.MakeGeometricCSpace(world, robot_idx);
      }else if(type=="TSE2"){
        cspace_level = factory.MakeKinodynamicCSpaceSE2(world, robot_idx);
      }else if(type=="TSE3"){
        cspace_level = factory.MakeKinodynamicCSpace(world, robot_idx);
      }else if(type=="R2T") {
        cspace_level = factory.MakeGeometricCSpaceRNTime(world, robot_idx, 2);
      }else if(type=="ANNULUS") {
        cspace_level = factory.MakeGeometricCSpaceAnnulus(world, robot_idx);
      }else if(type=="SOLIDCYLINDER") {
        cspace_level = factory.MakeGeometricCSpaceSolidCylinder(world, robot_idx);
      }else if(type=="SOLIDTORUS") {
        cspace_level = factory.MakeGeometricCSpaceSolidTorus(world, robot_idx);
      }else if(type=="MOBIUS") {
        cspace_level = factory.MakeGeometricCSpaceMobius(world, robot_idx);
      }else if(type=="CIRCULAR"){
        cspace_level = factory.MakeGeometricCSpaceCircular(world, robot_idx);
      }else{
        std::cout << std::string(80, '#') << std::endl;
        std::cout << "Type " << type << " not recognized" << std::endl;
        std::cout << std::string(80, '#') << std::endl;
        throw "Unrecognized type.";
      }
    }else{
      if(type.substr(0,1) == "R"){
        std::string str_dimension = type.substr(1);
        int N = boost::lexical_cast<int>(str_dimension);
        cspace_level = factory.MakeGeometricCSpaceFixedBase(world, robot_idx, N);
      }else if(type=="S1"){
        cspace_level = factory.MakeGeometricCSpaceSO2(world, robot_idx);
      }else if(type.substr(0,3) == "S1R"){
        // std::string str_dimension = type.substr(3);
        // int N = boost::lexical_cast<int>(str_dimension);
        cspace_level = factory.MakeGeometricCSpaceSO2RN(world, robot_idx);
      }else if(type=="TS1"){
        cspace_level = factory.MakeKinodynamicCSpaceSO2(world, robot_idx);
      }else{
        std::cout << type.substr(0) << std::endl;
        std::cout << "fixed robots needs to have configuration space RN or SN, but has " << type << std::endl;
        throw "Wrong Configuration Space.";
      }
    }
  }


  std::cout << "Create QuotientSpace with dimensionality " << cspace_level->GetDimensionality() << "[OMPL] and " 
    << cspace_level->GetKlamptDimensionality() << "[Klampt] (Robot Index " << cspace_level->GetRobotIndex() << ")." << std::endl;

  return cspace_level;
}

CSpaceOMPL* MotionPlanner::ComputeCSpaceLayer(const Layer &layer)
{

  if(!input.multiAgent){
    int ii = layer.inner_index;
    int io = layer.outer_index;
    Robot* ri = world->robots[ii];
    Robot* ro = world->robots[io];

    if(ri==nullptr){
      std::cout << "Robot " << ii << " does not exist." << std::endl;
      throw "Robot non-existent.";
    }
    if(ro==nullptr){
      std::cout << "Robot " << io << " does not exist." << std::endl;
      throw "Robot non-existent.";
    }

    std::string type = layer.type;
    CSpaceOMPL *cspace_layer = ComputeCSpace(type, ii, input.freeFloating);
    if(layer.finite_horizon_relaxation > 0){
      std::cout << "Finite Horizon relaxation: " << layer.finite_horizon_relaxation << std::endl;
    }
    return cspace_layer;
  }else{
    CSpaceOMPL *cspace_layer = ComputeMultiAgentCSpace(layer);
    return cspace_layer;
  }
}

void MotionPlanner::CreateHierarchy()
{
  hierarchy = std::make_shared<HierarchicalRoadmap>();
  std::string algorithm = input.name_algorithm;

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  //#########################################################################
  //For each level, compute the inputs/cspaces for each robot
  //#########################################################################

  if(util::StartsWith(algorithm, "hierarchy")){
    std::vector<Layer> layers = input.stratifications.front().layers;
    for(uint k = 0; k < layers.size(); k++){
      CSpaceOMPL *cspace_level_k = ComputeCSpaceLayer(layers.at(k));
      ob::State *stateTmp = cspace_level_k->SpaceInformationPtr()->allocState();

      cspace_levels.push_back( cspace_level_k );

      if(!layers.at(k).isMultiAgent){
        int io = layers.at(k).outer_index;
        int ii = layers.at(k).inner_index;
        uint N = cspace_level_k->GetKlamptDimensionality();
        // Config qi = input.q_init; qi.resize(N);
        // Config qg = input.q_goal; qg.resize(N);

        //############################################################################
        //SANITY CHECK
        Config qi_full = input.q_init; qi_full.resize(N);
        Config qg_full = input.q_goal; qg_full.resize(N);

        cspace_level_k->ConfigToOMPLState(qi_full, stateTmp);
        Config qi = cspace_level_k->OMPLStateToConfig(stateTmp);
        cspace_level_k->ConfigToOMPLState(qg_full, stateTmp);
        Config qg = cspace_level_k->OMPLStateToConfig(stateTmp);


        // bool invalid = ((qi - qi_proj).norm() > 1e-10);
        // invalid |= ((qg - qg_proj).norm() > 1e-10);

        // if(invalid){
        //   std::cout << "Error: Projected Config does not equal stated Config." << std::endl;
        //   std::cout << "Qinit (orig): " << qi << std::endl;
        //   std::cout << "Qinit (proj): " << qi_proj << std::endl;
        //   std::cout << "Qgoal (orig): " << qg << std::endl;
        //   std::cout << "Qgoal (proj): " << qg_proj << std::endl;
        //   throw "Error";
        // }
        //############################################################################

        layers.at(k).q_init = qi;
        layers.at(k).q_goal = qg;
        if(k==0){
          //add a root node
          hierarchy->AddLevel( ii, io, qi, qg);
        }
        hierarchy->AddLevel( ii, io, qi, qg); 
      }else{
        Config qi,qg;
        if(k>=layers.size()-1)
        {
          qi = input.q_init;
          qg = input.q_goal;
        }else{
          std::vector<int> Nklampts = 
            static_cast<CSpaceOMPLMultiAgent*>(cspace_level_k)->GetKlamptDimensionalities();
          for(uint j = 0; j < layers.back().q_inits.size(); j++){
            Config qinit = layers.back().q_inits.at(j);
            Config qgoal = layers.back().q_goals.at(j);
            input.AddConfigToConfig(qi, qinit, Nklampts.at(j));
            input.AddConfigToConfig(qg, qgoal, Nklampts.at(j));
          }
        }
        std::vector<int> idxs = static_cast<CSpaceOMPLMultiAgent*>(cspace_level_k)->GetRobotIdxs();

        Config qi_full = qi;
        Config qg_full = qg;

        cspace_level_k->ConfigToOMPLState(qi_full, stateTmp);
        qi = cspace_level_k->OMPLStateToConfig(stateTmp);
        cspace_level_k->ConfigToOMPLState(qg_full, stateTmp);
        qg = cspace_level_k->OMPLStateToConfig(stateTmp);

        if(k==0){
          //add a root node
          hierarchy->AddLevel( idxs, qi, qg);
        }
        hierarchy->AddLevel( idxs, qi, qg); 

      }
      cspace_level_k->SpaceInformationPtr()->freeState(stateTmp);
    }

    //empty roadmap on the highest level (so that we can collapse the whole
    //hierarchy into a singleton node)
    hierarchy->AddRootNode( std::make_shared<Roadmap>() ); 
    std::vector<int> path;
    for(uint k = 0; k < cspace_levels.size(); k++){
      hierarchy->AddNode( std::make_shared<Roadmap>(), path ); 
      path.push_back(0);
    }

    //Check if any levels have constraint relaxations
    for(uint k = 0; k < cspace_levels.size(); k++){
      double cr = layers.at(k).finite_horizon_relaxation;
      if(cr > 0){
          std::cout << "Constraint relaxation on level " << k << std::endl;
          Config qcr = layers.at(k).q_init;
          std::cout << qcr << " (" << cr << ")" << std::endl;
          CSpaceOMPL *cspace = cspace_levels.at(k);
          ob::State *xCenter = cspace->SpaceInformationPtr()->allocState();
          cspace->ConfigToOMPLState(qcr, xCenter);
          cspace_levels.at(k)->setStateValidityCheckerConstraintRelaxation(xCenter, cr);
      }
    }

  }else{

    Layer layer = input.stratifications.front().layers.back();
    CSpaceOMPL *cspace = ComputeCSpaceLayer(layer);
    cspace_levels.push_back(cspace);
    if(!layer.isMultiAgent){
      //shallow algorithm (use last robot in hierarchy)
      //two levels, so we can collapse the roadmap 
      uint N = cspace->GetRobotPtr()->q.size();
      Config qi = input.q_init; qi.resize(N);
      Config qg = input.q_goal; qg.resize(N);
      int ri = cspace->GetRobotIndex();
      hierarchy->AddLevel(ri, qi, qg);
      hierarchy->AddLevel(ri, qi, qg);

    }else{
      std::vector<int> idxs = static_cast<CSpaceOMPLMultiAgent*>(cspace)->GetRobotIdxs();
      hierarchy->AddLevel( idxs, input.q_init, input.q_goal);
      hierarchy->AddLevel( idxs, input.q_init, input.q_goal); 
    }
    hierarchy->AddRootNode( std::make_shared<Roadmap>() ); 
    std::vector<int> path;
    hierarchy->AddNode( std::make_shared<Roadmap>(), path ); 
  }

  if(util::StartsWith(algorithm, "benchmark") || 
     util::StartsWith(algorithm, "optimizer") 
     ){
    if(input.stratifications.empty()){
      OMPL_INFORM("Benchmark has no stratifications");
      return;
    }
    Layer last_layer = input.stratifications.at(0).layers.back();

    for(uint k = 0; k < input.stratifications.size(); k++){
      std::vector<Layer> layers = input.stratifications.at(k).layers;
      std::vector<CSpaceOMPL*> cspace_strat_k;
      for(uint j = 0; j < layers.size(); j++){
        CSpaceOMPL *cspace_strat_k_level_j = ComputeCSpaceLayer(layers.at(j));
        cspace_strat_k.push_back( cspace_strat_k_level_j );
      }
      //DEBUG
      //############################################################################
      // CSpaceOMPL *cspace_strat_k_last_level = ComputeCSpaceLayer(layers.back());
      // uint N_kl = cspace_strat_k_last_level->GetDimensionality();
      // uint N_a = cspace_ambient->GetDimensionality();
      // if(N_kl != N_a){
      //   OMPL_INFORM("For benchmark we require ALL hierarchies to share the same ambient space.");
      //   std::cout << "Please make sure every hierarchy has the same last entry" << std::endl;
      //   std::cout << "However, stratification " << k << " has Dimensionality " << N_kl << std::endl;
      //   std::cout << "While stratification 0 has Dimensionality " << N_a << std::endl;
      // }
      //############################################################################
      //every stratification must share the same ambient space
      //cspace_strat_k.push_back(cspace_ambient);
      cspace_stratifications.push_back(cspace_strat_k);
    }
  }
}

void MotionPlanner::Clear()
{
  if(!active) return;
  pwl = nullptr;
  current_level = 0;
  current_level_node = 0;
  current_path.clear();

  strategy->Clear();
  viewHierarchy.Clear();
}

void MotionPlanner::InitStrategy()
{
  StrategyInput strategy_input = input.GetStrategyInput();
  strategy_input.cspace_levels = cspace_levels;
  strategy_input.cspace_stratifications = cspace_stratifications;
  strategy->Init(strategy_input);
}

void MotionPlanner::Step()
{
  if(!active) return;
  current_level = 0;
  current_level_node = 0;
  current_path.clear();
  viewHierarchy.Clear();

  if(!strategy->IsInitialized()){
    InitStrategy();
  }

  StrategyOutput output(cspace_levels.back());
  resetTime();
  strategy->Step(output);
  time = getTime();
  output.GetHierarchicalRoadmap( hierarchy, cspace_levels );
}

void MotionPlanner::StepOneLevel()
{
  if(!active) return;
  if(cspace_levels.size()<=2){
    return AdvanceUntilSolution();
  }

  if(!strategy->IsInitialized()){
    current_level = 0;
    current_level_node = 0;
    current_path.clear();
    viewHierarchy.Clear();
    InitStrategy();
  }

  StrategyOutput output(cspace_levels.back());

  uint numberOfSolutionPathsCurrentLevel = 0;
  resetTime();
  while(numberOfSolutionPathsCurrentLevel < 1)
  {
    strategy->Step(output);
    output.GetHierarchicalRoadmap( hierarchy, cspace_levels );
    numberOfSolutionPathsCurrentLevel = hierarchy->NumberNodesOnLevel(current_level+2);
  }
  time = getTime();
}

void MotionPlanner::AdvanceUntilSolution()
{
  if(!active) return;

  if(!strategy->IsInitialized()){
    InitStrategy();
    current_level = 0;
    current_level_node = 0;
    current_path.clear();
    viewHierarchy.Clear();
  }else{
    // strategy->Clear();
  }
  resetTime();
  if(!util::StartsWith(input.name_algorithm,"benchmark")){
    StrategyOutput output(cspace_levels.back());
    strategy->Plan(output);
    output.GetHierarchicalRoadmap( hierarchy, cspace_levels );
    // std::cout << output << std::endl;
  }
  time = getTime();

  ExpandSimple();
}

PlannerInput& MotionPlanner::GetInput(){
  return input;
}
bool MotionPlanner::isActive(){
  return active;
}

void MotionPlanner::ExpandFull(){
  if(!active) return;

  uint Nmax=hierarchy->NumberLevels();

  current_level = 0;
  current_level_node = 0;
  current_path.clear();
  viewHierarchy.Clear();

  while(true){
    if(current_level<Nmax-1){
      if(hierarchy->HasChildren(current_path)){
        current_level++;
        current_level_node=hierarchy->NumberChildren(current_path)-1;
        current_path.push_back(current_level_node);
      }else{
        break;
      }
    }else{
      break;
    }
    UpdateHierarchy();
  }
  UpdateHierarchy();
}

//folder-like operations on hierarchical roadmap
void MotionPlanner::Expand(){
  if(!active) return;

  uint Nmax=hierarchy->NumberLevels();
  if(current_level<Nmax-1){
    if(hierarchy->HasChildren(current_path)){
      current_level++;
      current_level_node=hierarchy->NumberChildren(current_path)-1;
      current_path.push_back(current_level_node);
    }else{
      AdvanceUntilSolution();
    }
  }
  UpdateHierarchy();
}
void MotionPlanner::ExpandSimple(){
  if(!active) return;

  uint Nmax=hierarchy->NumberLevels();
  if(current_level<Nmax-1)
  {
    // std::cout << (hierarchy->HasChildren(current_path)?"HasChildren":"NoChildren") << std::endl;
    if(hierarchy->HasChildren(current_path))
    {
      current_level++;
      current_level_node=hierarchy->NumberChildren(current_path)-1;
      current_path.push_back(current_level_node);
    }
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
        // uint idx = hierarchy->GetRobotIdx( current_level );
        // Robot *robot = world->robots[idx];
        // uint N = hierarchy->NumberNodesOnLevel(current_level);
        // viewHierarchy.PushLevel(N, robot->name);
      uint N = hierarchy->NumberNodesOnLevel(current_level);
      viewHierarchy.PushLevel(N, "");
    }
  }
  pwl = GetPath();
  if(pwl && input.smoothPath){
    pwl->Smooth();
  }
  viewHierarchy.UpdateSelectionPath( current_path );
  setSelectedPath(current_path);
}

void MotionPlanner::setSelectedPath(std::vector<int> selectedPath)
{
    //can only be done with Explorer Planners
    auto selectionPlanner = dynamic_pointer_cast<og::MotionExplorer>(strategy->GetPlannerPtr());
    if(selectionPlanner != NULL){
      selectionPlanner->setSelectedPath( selectedPath );
    }
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
  if(!active) return nullptr;
  Rcurrent = hierarchy->GetNodeContent(current_path);
  pwl = Rcurrent->GetShortestPath();
  return pwl;
}

void MotionPlanner::DrawGL(GUIState& state){
  if(!active) return;


  uint Nsiblings;
  if(current_path.size()>1){
    std::vector<int>::const_iterator first = current_path.begin();
    std::vector<int>::const_iterator last = current_path.end()-1;
    std::vector<int> current_parent_path(first, last);
    Nsiblings = hierarchy->NumberChildren(current_parent_path);
  }else{
    Nsiblings = hierarchy->NumberNodesOnLevel(current_path.size());
  }

  if(current_path.size() > 0){
      int last_node = current_path.back();
      const GLColor magenta(0.7,0,0.7,1);

      const GLColor colorPathSelectedExecutable = green;
      const GLColor colorPathSelectedNonExec = magenta;
      const GLColor colorPathSelectedNonExecChildren = green;

      const GLColor colorPathNotSelected = lightGrey;
      const GLColor colorPathNotSelectedChildren = lightGrey;

      for(uint k = 0; k < Nsiblings; k++){
        if(k==(uint)last_node) continue;
        current_path.back() = k;
        Rcurrent = hierarchy->GetNodeContent(current_path);
        Rcurrent->DrawGL(state);
        PathPiecewiseLinear *pwlk = Rcurrent->GetShortestPath();
        bool hasChildren = hierarchy->HasChildren(current_path);
        if(pwlk && state("draw_roadmap_shortest_path")){
          pwlk->zOffset = 0.001;
          pwlk->linewidth = 0.3*input.pathWidth;
          pwlk->widthBorder= 0.3*input.pathBorderWidth;
          pwlk->ptsize = 8;
          if(!hasChildren){
              pwlk->setColor(colorPathNotSelected);
          }else{
              pwlk->setColor(colorPathNotSelectedChildren);

          }
          pwlk->drawSweptVolume = false;
          pwlk->drawCross = false;
          pwlk->DrawGL(state);
        }
      }

      current_path.back() = last_node;
      Rcurrent = hierarchy->GetNodeContent(current_path);

      //draw current selected roadmap
      Rcurrent->DrawGL(state);

      pwl = Rcurrent->GetShortestPath();
      if(pwl && state("draw_roadmap_shortest_path")){
        pwl->zOffset = 0.015;
        pwl->linewidth = input.pathWidth;
        pwl->widthBorder= input.pathBorderWidth;
        pwl->ptsize = 10;


        pwl->cRobotVolume = GLColor(0.8,0.8,0.8,1);
        pwl->drawSweptVolume = true;
        unsigned maxLevels = hierarchy->NumberLevels();
        unsigned curLevel = hierarchy->GetNode(current_path)->level;
        bool hasChildren = hierarchy->HasChildren(current_path);
        if(curLevel < maxLevels-1){
            pwl->drawCross = true;
            if(hasChildren){
                pwl->cCross = colorPathSelectedNonExecChildren;
                pwl->setColor(colorPathSelectedNonExecChildren);
            }else{
                pwl->cCross = colorPathSelectedNonExec;
                pwl->setColor(colorPathSelectedNonExec);
            }
        }else{
            pwl->drawCross = false;
            pwl->setColor(colorPathSelectedExecutable);
        }
        pwl->DrawGL(state);
      }
  }

  unsigned maxLevel = hierarchy->NumberLevels()-1;
  std::vector<int> ridx = hierarchy->GetRobotIdxs(current_level);
  std::vector<int> ridx_outer = hierarchy->GetRobotIdxs(maxLevel);

  std::vector<Robot*> robots;
  std::vector<Robot*> robots_outer;
  for(uint k = 0; k < ridx.size(); k++){
    if(ridx.at(k)>=0)
    robots.push_back( world->robots[ridx.at(k)] );
  }
  for(uint k = 0; k < ridx_outer.size(); k++){
    if(ridx_outer.at(k)>=0)
    robots_outer.push_back( world->robots[ridx_outer.at(k)] );
  }

  const Config qi = hierarchy->GetInitConfig(current_level);
  const Config qg = hierarchy->GetGoalConfig(current_level);

  const Config qiOuter = hierarchy->GetInitConfig(maxLevel);
  const Config qgOuter = hierarchy->GetGoalConfig(maxLevel);

  const GLColor ultralightred_sufficient(0.8,0,0,0.3);
  const GLColor ultralightgreen_sufficient(0,0.5,0,0.2);

  for(uint k = 0; k < cspace_levels.size(); k++)
  {
      CSpaceOMPL* ck = cspace_levels.at(k);
      ck->DrawGL(state);
  }

      // if(k < current_level)
      // {
      //     ob::State *qompl = ck->SpaceInformationPtr()->allocState();

      //     if(state("planner_draw_start_configuration"))
      //     {
      //         ck->ConfigToOMPLState(qi, qompl);
      //         Config qk = ck->OMPLStateToConfig(qompl);
      //         GLDraw::drawRobotsAtConfig(robots, qk, green);
      //     }
      //     if(state("planner_draw_goal_configuration"))
      //     {
      //         ck->ConfigToOMPLState(qg, qompl);
      //         Config qk = ck->OMPLStateToConfig(qompl);
      //         GLDraw::drawRobotsAtConfig(robots, qk, red);
      //     }

      //     ck->SpaceInformationPtr()->freeState(qompl);
      // }
  // }

  if(state("planner_draw_start_configuration")){
    GLDraw::drawRobotsAtConfig(robots, qi, green);
    if(state("planner_draw_start_goal_configuration_sufficient")){
      GLDraw::drawRobotsAtConfig(robots_outer, qiOuter, ultralightgreen_sufficient);
    }
  }
  if(state("planner_draw_goal_configuration")){
    GLDraw::drawRobotsAtConfig(robots, qg, red);
    if(state("planner_draw_start_goal_configuration_sufficient")){
      GLDraw::drawRobotsAtConfig(robots_outer, qgOuter, ultralightred_sufficient);
    }
  }

}
void MotionPlanner::resetTime()
{
  timePointStart = ompl::time::now();
}

double MotionPlanner::getTime()
{
  timePointEnd = ompl::time::now();
  ompl::time::duration timeDuration = timePointEnd - timePointStart;
  return ompl::time::seconds(timeDuration);
}

double MotionPlanner::getLastIterationTime()
{
  // int timeInt = (int)time*100.0;
  // time = ((double)timeInt/100.0);
  return time;
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
