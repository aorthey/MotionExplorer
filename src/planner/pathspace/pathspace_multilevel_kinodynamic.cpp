#include "pathspace_multilevel_kinodynamic.h"
#include "pathspace_atomic.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_kinodynamic.h"
#include "gui/drawMotionPlanner.h"
#include <boost/lexical_cast.hpp>


PathSpaceMultiLevelKinodynamic::PathSpaceMultiLevelKinodynamic(RobotWorld *world_, PathSpaceInput* input_):
  PathSpaceMultiLevel(world_, input_)
{
}

std::vector<PathSpace*> PathSpaceMultiLevelKinodynamic::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  std::vector<CSpaceOMPL*> cspace_levels;
  PathSpaceInput* input_level = input->GetNextLayer();
  PathSpaceInput* last_level = input_level;

  if(input->freeFloating){
    while(input_level){
      CSpaceOMPL *cspace_level_k;
      //if(input_level->type=="R3") {
        //cspace_level_k = factory.MakeKinodynamicCSpaceRN(world, input_level->robot_idx, 3);
      if(input_level->type=="SE3"){
        cspace_level_k = factory.MakeKinodynamicCSpace(world, input_level->robot_idx);
      }else{
        std::cout << "Type " << input_level->type << " not recognized" << std::endl;
        exit(0);
      }
      cspace_levels.push_back( cspace_level_k );
      last_level = input_level;
      input_level = input_level->GetNextLayer();
    }
  }else{
    exit(0);
  }

  StrategyKinodynamicMultiLevel strategy;
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
