#include "pathspace_ompl_twolevel.h"
#include "pathspace_atomic.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"

PathSpaceOMPLTwoLevel::PathSpaceOMPLTwoLevel(RobotWorld *world_, PathSpaceInput* input_):
  PathSpaceOMPL(world_, input_)
{
}
std::vector<PathSpace*> PathSpaceOMPLTwoLevel::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  int robot_idx = input->robot_idx;
  Robot *robot = world->robots[robot_idx];
  SingleRobotCSpace *kcspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);

  PathSpaceInput* next = input->GetNextLayer()->GetNextLayer();

  CSpaceOMPL *cspace = factory.MakeGeometricCSpace(robot, kcspace);
  cspace->print();

  StrategyGeometric strategy;
  StrategyOutput output(cspace);

  StrategyInput strategy_input = input->GetStrategyInput();
  strategy_input.cspace = cspace;

  robot_idx = next->robot_idx;
  robot = world->robots[robot_idx];
  SingleRobotCSpace *kcspace1 = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
  CSpaceOMPL* cspace_level1 = factory.MakeGeometricCSpace(robot, kcspace1);
  cspace_level1->print();

  strategy_input.cspace_level1 = cspace_level1;

  strategy.plan(strategy_input, output);

  std::vector<PathSpace*> decomposedspace;

  if(output.hasExactSolution()){
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.at(0)->SetShortestPath( output.GetShortestPath() );
    decomposedspace.at(0)->SetRoadmap( *output.GetRoadmapPtr() );
  }else{
    std::cout << "Error: Path could not be expanded" << std::endl;
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.at(0)->SetRoadmap( *output.GetRoadmapPtr() );
  }
  return decomposedspace;

}
