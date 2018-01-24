#include "pathspace_ompl.h"
#include "pathspace_atomic.h"

#include "planner/cspace/cspace_factory.h"
#include "planner/strategy/strategy_geometric.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"

PathSpaceOMPL::PathSpaceOMPL(RobotWorld *world_, PathSpaceInput* input_):
  PathSpace(world_, input_)
{
}

bool PathSpaceOMPL::isAtomic() const{
  return false;
}
std::vector<PathSpace*> PathSpaceOMPL::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());
  int robot_idx = input->robot_idx;
  CSpaceOMPL *cspace;
  if(input->freeFloating){
    cspace = factory.MakeGeometricCSpace(world, robot_idx);
  }else{
    cspace = factory.MakeGeometricCSpaceFixedBase(world, robot_idx);
  }

  StrategyGeometric strategy;
  StrategyOutput output(cspace);
  StrategyInput strategy_input = input->GetStrategyInput();
  strategy_input.cspace = cspace;
  strategy.plan(strategy_input, output);

  std::vector<PathSpace*> decomposedspace;

  if(output.hasExactSolution()){
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    //decomposedspace.at(0)->SetShortestPath( output.GetShortestPath() );
    decomposedspace.at(0)->SetShortestPath( output.getShortestPathOMPL(), cspace );
    decomposedspace.at(0)->SetRoadmap( output.GetRoadmapPtr() );
  }else{
    std::cout << "Error: Path could not be expanded" << std::endl;
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.at(0)->SetRoadmap( output.GetRoadmapPtr() );
  }
  return decomposedspace;

}
void PathSpaceOMPL::DrawGL(GUIState& state){
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi = input->q_init;
  const Config qg = input->q_goal;

  GLDraw::drawRobotAtConfig(robot, qi, lightGreen);
  GLDraw::drawRobotAtConfig(robot, qg, lightRed);
}
