#include "pathspace_multilevel_SE2.h"
#include "pathspace_atomic.h"
#include "elements/roadmap_decorator.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_geometric_multilevel.h"

PathSpaceMultiLevelSE2::PathSpaceMultiLevelSE2(RobotWorld *world_, PathSpaceInput* input_):
  PathSpaceOMPL(world_, input_)
{
}

std::vector<PathSpace*> PathSpaceMultiLevelSE2::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  std::vector<CSpaceOMPL*> cspace_levels;
  PathSpaceInput* input_level = input->GetNextLayer();

  CSpaceOMPL *cspace_level_k;
  while(input_level){
    std::cout << *input_level << std::endl;
    //uint k = input_level->level;
    if(input_level->type=="R2") {
      cspace_level_k = factory.MakeGeometricCSpaceRN(world, input_level->robot_idx, 2);
    }else if(input_level->type=="SE2"){
      cspace_level_k = factory.MakeGeometricCSpaceSE2(world, input_level->robot_idx);
    }else{
      std::cout << "Type " << input_level->type << " not recognized" << std::endl;
      exit(0);
    }
    cspace_levels.push_back( cspace_level_k );
    input_level = input_level->GetNextLayer();
  }

  StrategyGeometricMultiLevel strategy;
  StrategyOutput output(cspace_levels.back());
  StrategyInput strategy_input = input->GetStrategyInput();
  strategy_input.cspace = cspace_levels.back();
  strategy_input.world = world;

  //multilevel input
  strategy_input.cspace_levels = cspace_levels;

  strategy.plan(strategy_input, output);

  std::vector<PathSpace*> decomposedspace;
  decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
  RoadmapPtr roadmap1 = output.GetRoadmapPtr();
  //RoadmapDecoratorSE2Ptr roadmap( new RoadmapDecoratorSE2(roadmap1) );
  decomposedspace.back()->SetRoadmap( roadmap1 );

  std::cout << std::string(80, '-') << std::endl;
  std::cout << output << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  if(output.hasExactSolution()){
    std::vector<Config> path = output.GetShortestPath();
    //roadmap->SetShortestPath( path );
    //decomposedspace.at(0)->SetShortestPath( path );
    decomposedspace.at(0)->SetShortestPath( output.getShortestPathOMPL(), cspace_levels.back() );
  }
  return decomposedspace;

}
