#include "pathspace_multilevel_SE3.h"
#include "pathspace_atomic.h"
#include "elements/roadmap_decorator.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_geometric_multilevel.h"
#include <boost/lexical_cast.hpp>

PathSpaceMultiLevelSE3::PathSpaceMultiLevelSE3(RobotWorld *world_, PathSpaceInput* input_):
  PathSpaceOMPL(world_, input_)
{
}

std::vector<PathSpace*> PathSpaceMultiLevelSE3::Decompose(){
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
      if(input_level->type=="R3") {
        cspace_level_k = factory.MakeGeometricCSpaceRN(world, input_level->robot_idx, 3);
      }else if(input_level->type=="R3S2"){
        cspace_level_k = factory.MakeGeometricCSpaceR3S2(world, input_level->robot_idx);
      }else if(input_level->type=="SE3"){
        cspace_level_k = factory.MakeGeometricCSpaceSE3(world, input_level->robot_idx);
      }else if(input_level->type=="SE3RN"){
        cspace_level_k = factory.MakeGeometricCSpace(world, input_level->robot_idx);
      }else{
        std::cout << "Type " << input_level->type << " not recognized" << std::endl;
        exit(0);
      }
      if(input->enableSufficiency){
        cspace_level_k = new CSpaceOMPLDecoratorNecessarySufficient(cspace_level_k, input_level->robot_outer_idx);
      }
      std::cout << *input_level << std::endl;
      cspace_levels.push_back( cspace_level_k );
      last_level = input_level;
      input_level = input_level->GetNextLayer();
    }
  }else{
    while(input_level){
      //uint k = input_level->level;
      CSpaceOMPL *cspace_level_k;
      std::string str_dimension = input_level->type.substr(1);
      int N = boost::lexical_cast<int>(str_dimension);

      //cspace_level_k = factory.MakeGeometricCSpaceRN(world, input_level->robot_idx, N);
      cspace_level_k = factory.MakeGeometricCSpaceFixedBase(world, input_level->robot_idx, N);
      if(input->enableSufficiency){
        cspace_level_k = new CSpaceOMPLDecoratorNecessarySufficient(cspace_level_k, input_level->robot_outer_idx);
      }

      std::cout << *input_level << std::endl;
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

  //multilevel input
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
