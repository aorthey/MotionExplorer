#include "pathspace/pathspace_linear_hierarchy.h"
#include "pathspace_atomic.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_roadmap.h"
#include "planner/cspace/cspace_factory.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"

PathSpaceLinearHierarchy::PathSpaceLinearHierarchy(RobotWorld *world_, PathSpaceInput* input_):
  PathSpace(world_, input_)
{
}

bool PathSpaceLinearHierarchy::isAtomic() const{
  return false;
}
void PathSpaceLinearHierarchy::DrawGL(GUIState&){
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi = input->q_init;
  const Config qg = input->q_goal;

  GLDraw::drawRobotAtConfig(robot, qi, lightGreen);
  GLDraw::drawRobotAtConfig(robot, qg, lightRed);
}

std::vector<PathSpace*> PathSpaceLinearHierarchy::Decompose(){
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

  std::vector<Config> shortest_path1 = output.GetShortestPath();
  std::vector<std::vector<Config>> solution_paths = output.GetSolutionPaths();

  std::vector<PathSpace*> decomposedspace;
  for(uint k = 0; k < solution_paths.size() ; k++){
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.back()->SetShortestPath( solution_paths.at(k) );
  }
  //std::vector<Config> shortest_path2 = roadmap.GetShortestPath();
  //std::cout << std::string(80, '-') << std::endl;
  //for(uint k = 0; k < shortest_path1.size(); k++){
  //  std::cout << shortest_path1.at(k)   << std::endl;  
  //}
  //std::cout << std::string(80, '-') << std::endl;
  //for(uint k = 0; k < shortest_path2.size(); k++){
  //  std::cout << shortest_path2.at(k)   << std::endl;  
  //}
  //std::cout << std::string(80, '-') << std::endl;
  //exit(0);
  //bool done = true;
  //while(!done){
  //  //(1) get shortest path from roadmap


  //  //(2) verify shortest path by checking that it is feasible. 
  //  PathSpaceInput *level1 = input->GetNextLayer();

  //  //(3a) if feasible, return path, interpolate along sufficient edges
  //  //(3b) if not feasible, remove the infeasible vertex/edge and repeat

  //}

  //###########################################################################
  //for each necessary vertex, create a new underlying cspace and create a
  //roadmap. this roadmap is then sampled until we find a feasible point. this
  //vertex is then added to the sufficient_roadmap, marking a feasible vertex.
  //The underlying roadmap should still grow, because there might be
  //disconnected components for each vertex
  //###########################################################################


  if(output.success){

    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.back()->SetRoadmap( sufficient_roadmap );
    decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
    decomposedspace.back()->SetRoadmap( necessary_roadmap );
  }else{
    std::cout << "Error: Path could not be expanded" << std::endl;
  }
  return decomposedspace;

}
