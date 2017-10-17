#include "pathspace/pathspace_hierarchical_roadmap.h"
#include "pathspace_atomic.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_roadmap.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"

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

  //###########################################################################
  // Extract from the planner the roadmap, and decompose the roadmap into a
  // roadmap containing only sufficient vertices (sufficient_roadmap), and a
  // roadmap which contains only necessary but not sufficient vertices
  // (necessary_roadmap). 
  //###########################################################################
  Roadmap roadmap = output.GetRoadmap();
  roadmap.cVertex = green;
  roadmap.cEdge = green;

  Roadmap sufficient_roadmap;
  sufficient_roadmap.CreateFromPlannerDataOnlySufficient(output.GetPlannerDataPtr(), cspace);
  sufficient_roadmap.cVertex = green;
  sufficient_roadmap.cEdge = green;

  Roadmap necessary_roadmap;
  necessary_roadmap.CreateFromPlannerDataOnlyNecessary(output.GetPlannerDataPtr(), cspace);
  necessary_roadmap.cVertex = magenta;
  necessary_roadmap.cEdge = magenta;

  std::vector<Config> Vn = necessary_roadmap.GetVertices();
  //###########################################################################
  //for each necessary vertex, create a new underlying cspace and create a
  //roadmap. this roadmap is then sampled until we find a feasible point. this
  //vertex is then added to the sufficient_roadmap, marking a feasible vertex.
  //The underlying roadmap should still grow, because there might be
  //disconnected components for each vertex
  //###########################################################################
  PathSpaceInput *level1 = input->GetNextLayer();

  std::vector<CSpaceOMPL*> slice_spaces;
  std::vector<StrategyRoadmap*> slice_roadmaps;

  for(uint k = 0; k < Vn.size(); k++){
    //grow one roadmap for each vertex
    uint inner_index = level1->robot_inner_idx;
    uint outer_index = level1->robot_outer_idx;

    Robot* robot_inner = world->robots[inner_index];
    Robot* robot_outer = world->robots[outer_index];

    SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
    CSpaceOMPL *cspace_k = factory.MakeGeometricCSpacePointConstraintSO3(robot_inner, cspace_klampt_i, Vn.at(k));
    slice_spaces.push_back(cspace_k);
    StrategyRoadmap *strategy_k = new StrategyRoadmap(cspace_k);
    slice_roadmaps.push_back(strategy_k);
  }

  std::vector<PathSpace*> decomposedspace;

  if(output.success){
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.at(0)->SetRoadmap( sufficient_roadmap );
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.at(1)->SetRoadmap( necessary_roadmap );
  }else{
    std::cout << "Error: Path could not be expanded" << std::endl;
  }
  return decomposedspace;

}
