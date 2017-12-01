#include "pathspace_multilevel_se3.h"
#include "pathspace_atomic.h"
#include "elements/roadmap_decorator.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_geometric_multilevel.h"

PathSpaceMultiLevelSE3::PathSpaceMultiLevelSE3(RobotWorld *world_, PathSpaceInput* input_):
  PathSpaceOMPL(world_, input_)
{
}

std::vector<PathSpace*> PathSpaceMultiLevelSE3::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  PathSpaceInput* next = input->GetNextLayer()->GetNextLayer();

  int robot_idx = input->robot_idx;
  CSpaceOMPL *cspace_level0 = factory.MakeGeometricCSpaceRN(world, robot_idx, 3);

  robot_idx = next->robot_idx;
  CSpaceOMPL* cspace_level1 = factory.MakeGeometricCSpaceSE3(world, robot_idx);

  StrategyGeometricMultiLevel strategy;
  StrategyOutput output(cspace_level1);
  StrategyInput strategy_input = input->GetStrategyInput();
  strategy_input.cspace = cspace_level1;
  strategy_input.world = world;

  //multilevel input
  strategy_input.cspace_level0 = cspace_level0;
  strategy_input.cspace_level1 = cspace_level1; //the complete original cspace

  strategy.plan(strategy_input, output);

  std::vector<PathSpace*> decomposedspace;
  decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
  RoadmapPtr roadmap = output.GetRoadmapPtr();
  decomposedspace.back()->SetRoadmap( roadmap );

  std::cout << std::string(80, '-') << std::endl;
  std::cout << output << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  if(output.hasExactSolution()){
    std::vector<Config> path = output.GetShortestPath();
    for(uint k = 0; k < path.size(); k++){
      std::cout << path.at(k) << std::endl;
    }
    roadmap->SetShortestPath( path );
    decomposedspace.at(0)->SetShortestPath( path );
  }
  return decomposedspace;

}
