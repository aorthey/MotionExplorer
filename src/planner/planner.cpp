#include "planner/planner.h"
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_kinodynamic.h"
#include "planner/cspace/cspace_factory.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "gui/drawMotionPlanner.h"
#include "util.h"
#include "common.h"
#include "gui/ViewLocalMinimaTree.h"
#include <ompl/multilevel/planners/explorer/MotionExplorer.h>
#include <ompl/multilevel/planners/explorer/MotionExplorerQMP.h>
#include <ompl/multilevel/planners/explorer/datastructures/MultiLevelPathSpace.h>
#include <ompl/multilevel/planners/explorer/datastructures/LocalMinimaTree.h>

#include <boost/lexical_cast.hpp>
#include <thread>

using namespace GLDraw;

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  pwl = nullptr;
  active = true;
  current_level = 0;
  max_levels_ = 0;
  threading = input.threading;
  std::cout << "THREADING: "<< (threading?"ON":"OFF") << std::endl;

  // current_level_node = 0;
  // current_path.clear();
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

bool MotionPlanner::hasChanged()
{
  if(hasLocalMinimaTree())
  {
      return localMinimaTree_->hasChanged();
  }
  return false;
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
  CSpaceFactory factory(input.GetCSpaceInput(robot_idx));

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
      }else if(type=="SE3C"){
        cspace_level = factory.MakeGeometricCSpaceSE3Constrained(world, robot_idx);
      }else if(type=="SE3"){
        cspace_level = factory.MakeGeometricCSpaceSE3(world, robot_idx);
      }else if(type=="R3_CONTACT") {
        cspace_level = factory.MakeGeometricCSpaceContact3D(world, robot_idx);
      }else if(type=="R2_CONTACT") {
        cspace_level = factory.MakeGeometricCSpaceContact2D(world, robot_idx);
      }else if(type=="SE2"){
        cspace_level = factory.MakeGeometricCSpaceSE2(world, robot_idx);
      }else if(type=="SE2RN"){
        cspace_level = factory.MakeGeometricCSpaceSE2RN(world, robot_idx);
      }else if(type=="SE3RN"){
        cspace_level = factory.MakeGeometricCSpace(world, robot_idx);
      }else if(type=="TSE2"){
        cspace_level = factory.MakeKinodynamicCSpaceSE2(world, robot_idx);
      }else if(type=="SE3_DUBIN"){
        cspace_level = factory.MakeGeometricCSpaceSE3Dubin(world, robot_idx);
      }else if(type=="SE2_DUBIN"){
        cspace_level = factory.MakeGeometricCSpaceSE2Dubin(world, robot_idx);
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


  return cspace_level;
}

CSpaceOMPL* MotionPlanner::ComputeCSpaceLayer(const Layer &layer)
{

  CSpaceOMPL *cspace_layer = nullptr;

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
    cspace_layer = ComputeCSpace(type, ii, input.freeFloating);
    if(layer.finite_horizon_relaxation > 0){
      std::cout << "Finite Horizon relaxation: " << layer.finite_horizon_relaxation << std::endl;
    }

  }else{
    cspace_layer = ComputeMultiAgentCSpace(layer);
  }
  if(cspace_layer != nullptr)
  {
    std::cout << "Create QuotientSpace with dimensionality " 
      << cspace_layer->GetDimensionality() << "[OMPL] and " 
      << cspace_layer->GetKlamptDimensionality() << "[Klampt] (Robot Index ";
      if(!input.multiAgent){
        std::cout << cspace_layer->GetRobotIndex();
      }else{
        std::cout << static_cast<CSpaceOMPLMultiAgent*>(cspace_layer)->GetRobotIdxs();
      }
      std::cout << ")." << std::endl;
  }
  return cspace_layer;
}

void MotionPlanner::CreateHierarchy()
{
  // hierarchy = std::make_shared<HierarchicalRoadmap>();
  std::string algorithm = input.name_algorithm;

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  //#########################################################################
  //For each level, compute the inputs/cspaces for each robot
  //#########################################################################

  if(util::StartsWith(algorithm, "hierarchy"))
  {
    std::vector<Layer> layers = input.stratifications.front().layers;
    max_levels_ = layers.size()-1;
    for(uint k = 0; k < layers.size(); k++)
    {
      CSpaceOMPL *cspace_level_k = ComputeCSpaceLayer(layers.at(k));
      ob::State *stateTmp = cspace_level_k->SpaceInformationPtr()->allocState();

      cspace_levels.push_back( cspace_level_k );

      Config qi;
      Config qg;

      if(!layers.at(k).isMultiAgent){
        uint N = cspace_level_k->GetKlamptDimensionality();

        Config qi_full = input.q_init; qi_full.resize(N);
        Config qg_full = input.q_goal; qg_full.resize(N);

        cspace_level_k->ConfigToOMPLState(qi_full, stateTmp);
        qi = cspace_level_k->OMPLStateToConfig(stateTmp);
        cspace_level_k->ConfigToOMPLState(qg_full, stateTmp);
        qg = cspace_level_k->OMPLStateToConfig(stateTmp);

        layers.at(k).q_init = qi;
        layers.at(k).q_goal = qg;
      }else{
        if(k>=layers.size()-1)
        {
          qi = input.q_init;
          qg = input.q_goal;
          uint N = cspace_level_k->GetKlamptDimensionality();
          if(qi.size() != (int)N)
          {
            OMPL_ERROR("Init vector contains %d dimensions, but robot %d has %d dimensions", 
                qi.size(), cspace_level_k->GetRobotIndex(), N);
            throw "";
          }
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

      }
      config_init_levels.push_back(qi);
      config_goal_levels.push_back(qg);
      cspace_level_k->SpaceInformationPtr()->freeState(stateTmp);
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
    max_levels_ = 0;

    ob::State *stateTmp = cspace->SpaceInformationPtr()->allocState();

    Config qi = input.q_init;
    Config qg = input.q_goal;

    auto *cspaceContact  = dynamic_cast<GeometricCSpaceContact*>(cspace);
    if(cspaceContact!=nullptr)
    {
      cspaceContact->setInitialConstraints();
      cspaceContact->ConfigToOMPLState(qi, stateTmp);
      qi = cspaceContact->OMPLStateToConfig(stateTmp);

      cspaceContact->setGoalConstraints();
      cspaceContact->ConfigToOMPLState(qg, stateTmp);
      qg = cspaceContact->OMPLStateToConfig(stateTmp);

      //reset
      cspaceContact->setInitialConstraints();
    }else{
      cspace->ConfigToOMPLState(qi, stateTmp);
      qi = cspace->OMPLStateToConfig(stateTmp);
      cspace->ConfigToOMPLState(qg, stateTmp);
      qg = cspace->OMPLStateToConfig(stateTmp);
    }

    config_init_levels.push_back(qi);
    config_goal_levels.push_back(qg);

    cspace->SpaceInformationPtr()->freeState(stateTmp);
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
      cspace_stratifications.push_back(cspace_strat_k);
    }
  }
}

void MotionPlanner::Clear()
{
  if(!active) return;

  if(threadRunning)
  {
    OMPL_WARN("Tried Clearing planner, but thread still running.");
    return;
  }

  pwl = nullptr;
  current_level = 0;
  strategy->Clear();
}

void MotionPlanner::InitStrategy()
{
  StrategyInput strategy_input = input.GetStrategyInput();
  strategy_input.cspace_levels = cspace_levels;
  strategy_input.cspace_stratifications = cspace_stratifications;
  strategy->Init(strategy_input);
  auto explorerPlanner = 
    dynamic_pointer_cast<ompl::multilevel::MotionExplorer>(strategy->GetPlannerPtr());
  if(explorerPlanner != nullptr)
  {
      localMinimaTree_ = explorerPlanner->getLocalMinimaTree();
      viewLocalMinimaTree_ = std::make_shared<ViewLocalMinimaTree>(localMinimaTree_, cspace_levels);
  }
  auto explorerPlanner2 = 
    dynamic_pointer_cast<ompl::multilevel::MotionExplorerQMP>(strategy->GetPlannerPtr());
  if(explorerPlanner2 != nullptr)
  {
      localMinimaTree_ = explorerPlanner2->getLocalMinimaTree();
      viewLocalMinimaTree_ = std::make_shared<ViewLocalMinimaTree>(localMinimaTree_, cspace_levels);
  }
}

void MotionPlanner::Step()
{
  if(!active) return;
  current_level = 0;

  if(!strategy->IsInitialized())
  {
    InitStrategy();
  }

  output = new StrategyOutput(cspace_levels);
  resetTime();
  strategy->Step(*output);
  time = getTime();
}

void RunPlanner(StrategyPtr strategy, StrategyOutput* output, std::atomic<bool>& threadRunning)
{
    strategy->Plan(*output);
    threadRunning = false;
}

bool MotionPlanner::isRunning()
{
  return threadRunning;
}

void MotionPlanner::AdvanceUntilSolution()
{
  if(!active) return;
  if(threadRunning)
  {
    OMPL_ERROR("Tried Restarting planner, but thread still running.");
    return;
  }

  if(!strategy->IsInitialized()){
    InitStrategy();
  }
  if(util::StartsWith(input.name_algorithm,"benchmark")) return;

  if(threading){
    output = new StrategyOutput(cspace_levels);
    threadRunning = true;
    std::thread threadStrategy(RunPlanner, std::ref(strategy), std::ref(output), std::ref(threadRunning));
    threadStrategy.detach();
  }else{
      output = new StrategyOutput(cspace_levels);
      strategy->Plan(*output);
  }
}


PlannerInput& MotionPlanner::GetInput()
{
  return input;
}

bool MotionPlanner::isActive()
{
  return active;
}

void MotionPlanner::Expand()
{
  if(!active) return;
  if(hasLocalMinimaTree())
  {
    localMinimaTree_->setSelectedMinimumExpand();
  }else
  {
    if(current_level < max_levels_)
    {
      current_level++;
    }
  }
}

void MotionPlanner::Collapse()
{
  if(!active) return;
  if(hasLocalMinimaTree())
  {
      localMinimaTree_->setSelectedMinimumCollapse();
      return;
  }else{
    if(current_level > 0)
    {
      current_level--;
    }
  }
}

void MotionPlanner::Next()
{
  if(!active) return;
  if(!hasLocalMinimaTree()) return;
  localMinimaTree_->setSelectedMinimumNext();
}

void MotionPlanner::Previous()
{
  if(!active) return;
  if(!hasLocalMinimaTree()) return;
  localMinimaTree_->setSelectedMinimumPrev();
}

void MotionPlanner::Print()
{
  std::cout << std::string(80, '-') << std::endl;
}

bool MotionPlanner::hasLocalMinimaTree()
{
  return (localMinimaTree_ != nullptr);
}

void MotionPlanner::DrawGLScreen(double x_, double y_)
{
  if(!active) return;
  if(!hasLocalMinimaTree()) return;
  viewLocalMinimaTree_->DrawGLScreen(x_, y_);
}

CSpaceOMPL* MotionPlanner::GetCSpace()
{
  return cspace_levels.back();
}

PathPiecewiseLinear* MotionPlanner::GetPath()
{
  if(!active) return nullptr;
  if(hasLocalMinimaTree())
  {
      return viewLocalMinimaTree_->getPathSelected();
  }else{
    if(output != nullptr)
    {
      if(!hasLocalMinimaTree())
      {
          return output->getSolutionPath(current_level);
      }
    }
    return nullptr;
  }
}

void MotionPlanner::DrawGL(GUIState& state)
{
  if(!active) return;

  CSpaceOMPL *cspace = cspace_levels.at(current_level);
  std::lock_guard<std::recursive_mutex> guard(cspace->getLock());

  if(hasLocalMinimaTree())
  {
      std::lock_guard<std::recursive_mutex> guard(localMinimaTree_->getLock());
      viewLocalMinimaTree_->DrawGL(state);
      DrawGLStartGoal(state);
  }else{
      DrawGLStartGoal(state);
  }

  if(output != nullptr)
  {
      output->DrawGL(state, current_level);
      if(!hasLocalMinimaTree())
      {
        PathPiecewiseLinear *pwl = output->getSolutionPath(current_level);
        if(pwl != nullptr)
        {
            pwl->linewidth = input.pathWidth;
            pwl->widthBorder= input.pathBorderWidth;
            pwl->zOffset = 0.001;
            pwl->ptsize = 8;
            pwl->drawSweptVolume = false;
            pwl->drawCross = false;
            const GLColor magenta(0.7,0,0.7,1);
            pwl->setColor(magenta);
            pwl->DrawGL(state);
        }
      }
  }
  // DrawGLStartGoal(state);

  //uint Nsiblings;
  //if(current_path.size()>1){
  //  std::vector<int>::const_iterator first = current_path.begin();
  //  std::vector<int>::const_iterator last = current_path.end()-1;
  //  std::vector<int> current_parent_path(first, last);
  //  Nsiblings = hierarchy->NumberChildren(current_parent_path);
  //}else{
  //  Nsiblings = hierarchy->NumberNodesOnLevel(current_path.size());
  //}

  //if(current_path.size() > 0){
  //    int last_node = current_path.back();
  //    const GLColor magenta(0.7,0,0.7,1);

  //    const GLColor colorPathSelectedExecutable = green;
  //    const GLColor colorPathSelectedNonExec = magenta;
  //    const GLColor colorPathSelectedNonExecChildren = green;

  //    const GLColor colorPathNotSelected = lightGrey;
  //    const GLColor colorPathNotSelectedChildren = lightGrey;

  //    if(state("draw_explorer_non_selected_paths"))
  //    {
  //      for(uint k = 0; k < Nsiblings; k++){
  //        if(k==(uint)last_node) continue;
  //        current_path.back() = k;
  //        Rcurrent = hierarchy->GetNodeContent(current_path);
  //        Rcurrent->DrawGL(state);
  //        PathPiecewiseLinear *pwlk = Rcurrent->GetShortestPath();
  //        bool hasChildren = hierarchy->HasChildren(current_path);
  //        if(pwlk && state("draw_roadmap_shortest_path")){
  //          pwlk->zOffset = 0.001;
  //          pwlk->linewidth = 0.3*input.pathWidth;
  //          pwlk->widthBorder= 0.3*input.pathBorderWidth;
  //          pwlk->ptsize = 8;
  //          if(!hasChildren){
  //              pwlk->setColor(colorPathNotSelected);
  //          }else{
  //              pwlk->setColor(colorPathNotSelectedChildren);

  //          }
  //          pwlk->drawSweptVolume = false;
  //          pwlk->drawCross = false;
  //          pwlk->DrawGL(state);
  //        }
  //      }
  //    }

  //    current_path.back() = last_node;
  //    Rcurrent = hierarchy->GetNodeContent(current_path);

  //    //draw current selected roadmap
  //    Rcurrent->DrawGL(state);

  //    pwl = Rcurrent->GetShortestPath();
  //    if(pwl && state("draw_roadmap_shortest_path")){
  //      pwl->zOffset = 0.015;
  //      pwl->linewidth = input.pathWidth;
  //      pwl->widthBorder= input.pathBorderWidth;
  //      pwl->ptsize = 10;


  //      pwl->cRobotVolume = GLColor(0.8,0.8,0.8,1);
  //      pwl->drawSweptVolume = true;
  //      unsigned maxLevels = hierarchy->NumberLevels();
  //      unsigned curLevel = hierarchy->GetNode(current_path)->level;
  //      bool hasChildren = hierarchy->HasChildren(current_path);
  //      if(curLevel < maxLevels-1){
  //          pwl->drawCross = false;
  //          if(hasChildren){
  //              pwl->cCross = colorPathSelectedNonExecChildren;
  //              pwl->setColor(colorPathSelectedNonExecChildren);
  //          }else{
  //              pwl->cCross = colorPathSelectedNonExec;
  //              pwl->setColor(colorPathSelectedNonExec);
  //          }
  //      }else{
  //          pwl->drawCross = false;
  //          pwl->setColor(colorPathSelectedExecutable);
  //      }
  //      pwl->DrawGL(state);
  //    }
  //}

}
void MotionPlanner::DrawGLStartGoal(GUIState& state)
{
  for(uint k = 0; k < cspace_levels.size(); k++)
  {
      CSpaceOMPL* ck = cspace_levels.at(k);
      ck->DrawGL(state);
  }

  const Config qi = config_init_levels.at(current_level);
  const Config qg = config_goal_levels.at(current_level);

  const Config qiOuter = config_init_levels.at(max_levels_);
  const Config qgOuter = config_goal_levels.at(max_levels_);

  const GLColor ultralightred_sufficient(0.8,0,0,0.3);
  const GLColor ultralightgreen_sufficient(0,0.5,0,0.2);

  const GLColor colorGoalConfiguration = GLDraw::getColorRobotGoalConfiguration();
  const GLColor colorStartConfiguration = GLDraw::getColorRobotStartConfiguration();
  const GLColor colorGoalConfigurationTransparent = GLDraw::getColorRobotGoalConfigurationTransparent();
  const GLColor colorStartConfigurationTransparent = GLDraw::getColorRobotStartConfigurationTransparent();

  CSpaceOMPL* qspace = cspace_levels.at(current_level);
  CSpaceOMPL* cspace = cspace_levels.at(max_levels_);
  if(state("planner_draw_start_configuration")){
    qspace->drawConfig(qi, colorStartConfiguration);
    cspace->drawConfig(qiOuter, colorStartConfigurationTransparent);
  }
  if(state("planner_draw_goal_configuration")){
    qspace->drawConfig(qg, colorGoalConfiguration);
    cspace->drawConfig(qgOuter, colorGoalConfigurationTransparent);
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
  return time;
}

std::ostream& operator<< (std::ostream& out, const MotionPlanner& planner)
{
  out << std::string(80, '-') << std::endl;
  out << " Planner: " << std::endl;
  out << std::string(80, '-') << std::endl;
  out << " Robots  " << std::endl;
  for(uint k = 0; k < planner.cspace_levels.size(); k++){
    std::cout << "Level:" << k << std::endl;
  }
  out << std::string(80, '-') << std::endl;
  return out;
}
