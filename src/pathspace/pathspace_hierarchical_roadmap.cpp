#include "pathspace/pathspace_hierarchical_roadmap.h"

#include "pathspace_atomic.h"
#include "planner/cspace_factory.h"
#include "planner/strategy_geometric.h"
#include "gui/drawMotionPlanner.h"

PathSpaceHierarchicalRoadmap::PathSpaceHierarchicalRoadmap(RobotWorld *world_, PathSpaceInput* input_):
  PathSpace(world_, input_)
{
}

bool PathSpaceHierarchicalRoadmap::isAtomic() const{
  return false;
}
void PathSpaceHierarchicalRoadmap::DrawGL(GUIState&){
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi = input->q_init;
  const Config qg = input->q_goal;

  GLColor lightGrey(0.4,0.4,0.4,0.2);
  GLColor lightGreen(0.2,0.9,0.2,0.2);
  GLColor lightRed(0.9,0.2,0.2,0.2);
  GLDraw::drawRobotAtConfig(robot, qi, lightGreen);
  GLDraw::drawRobotAtConfig(robot, qg, lightRed);
}

std::vector<PathSpace*> PathSpaceHierarchicalRoadmap::Decompose(){
  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  uint inner_index = input->robot_inner_idx;
  uint outer_index = input->robot_outer_idx;

  Robot* robot_inner = world->robots[inner_index];
  Robot* robot_outer = world->robots[outer_index];

  SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
  SingleRobotCSpace* cspace_klampt_o = new SingleRobotCSpace(*world,outer_index,&worldsettings);

  CSpaceOMPL* cspace = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);
  cspace = factory.MakeCSpaceDecoratorNecessarySufficient(cspace, cspace_klampt_o);
  cspace->print();

  StrategyGeometric strategy;
  StrategyOutput output(cspace);
  strategy.plan(input->GetStrategyInput(), cspace, output);

  Roadmap roadmap = output.GetRoadmap();
  Roadmap sufficient_roadmap;
  sufficient_roadmap.CreateFromPlannerDataOnlySufficient(pd, cspace);

  std::vector<PathSpace*> decomposedspace;

  if(output.success){
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.at(0)->SetShortestPath( output.GetShortestPath() );
    //decomposedspace.at(0)->SetRoadmap( output.GetRoadmap() );
    decomposedspace.at(0)->SetRoadmap( sufficient_roadmap );
  }else{
    std::cout << "Error: Path could not be expanded" << std::endl;
  }
  return decomposedspace;
}
