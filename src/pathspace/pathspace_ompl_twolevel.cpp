#include "pathspace_ompl_twolevel.h"
#include "pathspace_atomic.h"
#include "elements/roadmap_decorator.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_geometric_multilevel.h"

PathSpaceOMPLTwoLevel::PathSpaceOMPLTwoLevel(RobotWorld *world_, PathSpaceInput* input_):
  PathSpaceOMPL(world_, input_)
{
}
std::vector<PathSpace*> PathSpaceOMPLTwoLevel::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  /// SE(2)
  PathSpaceInput* next = input->GetNextLayer()->GetNextLayer();
  int robot_idx = next->robot_idx;
  Robot *robot = world->robots[robot_idx];
  SingleRobotCSpace *kspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
  CSpaceOMPL* cspace = factory.MakeGeometricCSpaceSE2(robot, kspace);

  /// R^2
  robot_idx = input->robot_idx;
  robot = world->robots[robot_idx];
  SingleRobotCSpace *kcspace0 = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
  CSpaceOMPL *cspace_level0 = factory.MakeGeometricCSpaceR2(robot, kcspace0);

  StrategyGeometricMultiLevel strategy;
  StrategyOutput output(cspace);
  StrategyInput strategy_input = input->GetStrategyInput();
  strategy_input.cspace = cspace;
  strategy_input.world = world;

  //multilevel input
  strategy_input.cspace_level0 = cspace_level0;
  strategy_input.cspace_level1 = cspace;

  strategy.plan(strategy_input, output);

  std::vector<PathSpace*> decomposedspace;
  decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
  RoadmapPtr roadmap1 = output.GetRoadmapPtr();
  RoadmapDecoratorSE2Ptr roadmap( new RoadmapDecoratorSE2(roadmap1) );
  decomposedspace.back()->SetRoadmap( roadmap );

  if(output.hasExactSolution()){
    decomposedspace.back()->SetShortestPath( output.GetShortestPath() );
  }
  return decomposedspace;

}
