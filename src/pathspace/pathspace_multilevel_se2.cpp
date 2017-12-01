#include "pathspace_multilevel_se2.h"
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

  /// SE(2)
  PathSpaceInput* next = input->GetNextLayer()->GetNextLayer();
  int robot_idx = next->robot_idx;
  //Robot *robot = world->robots[robot_idx];
  //SingleRobotCSpace *kspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
  CSpaceOMPL* cspace_level1 = factory.MakeGeometricCSpaceSE2(world, robot_idx);

  /// R^2
  robot_idx = input->robot_idx;
  //robot = world->robots[robot_idx];
  //SingleRobotCSpace *kcspace0 = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
  CSpaceOMPL *cspace_level0 = factory.MakeGeometricCSpaceRN(world, robot_idx, 2);

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
  RoadmapPtr roadmap1 = output.GetRoadmapPtr();
  RoadmapDecoratorSE2Ptr roadmap( new RoadmapDecoratorSE2(roadmap1) );
  decomposedspace.back()->SetRoadmap( roadmap );

  std::cout << std::string(80, '-') << std::endl;
  std::cout << output << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  if(output.hasExactSolution()){
    std::vector<Config> path = output.GetShortestPath();
    roadmap->SetShortestPath( path );
    decomposedspace.at(0)->SetShortestPath( path );
  }
  return decomposedspace;

}
