#include "pathspace_multilevel.h"
#include "pathspace_atomic.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"
#include "gui/drawMotionPlanner.h"
#include <boost/lexical_cast.hpp>


PathSpaceMultiLevel::PathSpaceMultiLevel(RobotWorld *world_, PathSpaceInput* input_):
  PathSpace(world_, input_)
{
}

void PathSpaceMultiLevel::DrawGL(GUIState& state)
{
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi = input->q_init;
  const Config qg = input->q_goal;
  if(state("planner_draw_start_configuration")) GLDraw::drawRobotAtConfig(robot, qi, green);
  if(state("planner_draw_goal_configuration")) GLDraw::drawRobotAtConfig(robot, qg, red);
}

bool PathSpaceMultiLevel::isAtomic() const{
  return false;
}

std::vector<PathSpace*> PathSpaceMultiLevel::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  std::vector<CSpaceOMPL*> cspace_levels;
  PathSpaceInput* input_level = input->GetNextLayer();
  PathSpaceInput* last_level = input_level;

  if(input->freeFloating){
    while(input_level){
      //uint k = input_level->level;
      CSpaceOMPL *cspace_level_k;
      if(input_level->type=="R2") {
        cspace_level_k = factory.MakeGeometricCSpaceRN(world, input_level->robot_idx, 2);
      }else if(input_level->type=="R3") {
        cspace_level_k = factory.MakeGeometricCSpaceRN(world, input_level->robot_idx, 3);
      }else if(input_level->type=="R3S2"){
        cspace_level_k = factory.MakeGeometricCSpaceR3S2(world, input_level->robot_idx);
      }else if(input_level->type=="SE3"){
        cspace_level_k = factory.MakeGeometricCSpaceSE3(world, input_level->robot_idx);
      }else if(input_level->type=="SE2"){
        cspace_level_k = factory.MakeGeometricCSpaceSE2(world, input_level->robot_idx);
      }else if(input_level->type=="SE3RN"){
        cspace_level_k = factory.MakeGeometricCSpace(world, input_level->robot_idx);
      }else{
        std::cout << "Type " << input_level->type << " not recognized" << std::endl;
        exit(0);
      }
      if(input->enableSufficiency){
        cspace_level_k = new CSpaceOMPLDecoratorNecessarySufficient(cspace_level_k, input_level->robot_outer_idx);
      }
      //std::cout << *input_level << std::endl;
      cspace_levels.push_back( cspace_level_k );
      last_level = input_level;
      input_level = input_level->GetNextLayer();
    }
  }else{
    while(input_level){
      CSpaceOMPL *cspace_level_k;

      if(input_level->type.substr(0,1) != "R"){
        std::cout << input_level->type.substr(0) << std::endl;
        std::cout << "fixed robots needs to have configuration space RN, but has " << input_level->type << std::endl;
        exit(0);
      }

      std::string str_dimension = input_level->type.substr(1);
      int N = boost::lexical_cast<int>(str_dimension);

      cspace_level_k = factory.MakeGeometricCSpaceFixedBase(world, input_level->robot_idx, N);
      if(input->enableSufficiency){
        cspace_level_k = new CSpaceOMPLDecoratorNecessarySufficient(cspace_level_k, input_level->robot_outer_idx);
      }

      //std::cout << *input_level << std::endl;
      cspace_levels.push_back( cspace_level_k );
      last_level = input_level;
      input_level = input_level->GetNextLayer();
    }
  }

  StrategyGeometricMultiLevel strategy;
  StrategyOutput output(cspace_levels.back());
  StrategyInput strategy_input = last_level->GetStrategyInput();
  strategy_input.cspace = cspace_levels.back();
  strategy_input.world = world;

  strategy_input.cspace_levels = cspace_levels;

  strategy.plan(strategy_input, output);

  std::vector<PathSpace*> decomposedspace;
  decomposedspace.push_back( new PathSpaceAtomic(world, last_level) );
  RoadmapPtr roadmap = output.GetRoadmapPtr();
  decomposedspace.back()->SetRoadmap( roadmap );

  std::cout << std::string(80, '-') << std::endl;
  std::cout << output << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  if(output.hasExactSolution()){
    std::vector<Config> path = output.GetShortestPath();
    decomposedspace.at(0)->SetShortestPath( output.getShortestPathOMPL(), cspace_levels.back() );
  }
  return decomposedspace;

}
