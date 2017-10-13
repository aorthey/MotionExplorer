#include "pathspace_onetopic_cover.h"
#include "pathspace_atomic.h"
#include "drawMotionPlanner.h"
#include "planner/cspace_factory.h"
#include "planner/strategy_geometric.h"
#include "algorithms/onetopic_reduction.h"

PathSpaceOnetopicCover::PathSpaceOnetopicCover(RobotWorld *world_, PathSpaceInput* input_):
  PathSpace(world_, input_)
{
}

bool PathSpaceOnetopicCover::isAtomic() const{
  return false;
}

std::vector<PathSpace*> PathSpaceOnetopicCover::Decompose(){

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input->GetCSpaceInput());

  uint inner_index = input->robot_inner_idx;
  uint outer_index = input->robot_outer_idx;

  Robot* robot_inner = world->robots[inner_index];
  Robot* robot_outer = world->robots[outer_index];

  SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
  SingleRobotCSpace* cspace_klampt_o = new SingleRobotCSpace(*world,outer_index,&worldsettings);

  CSpaceOMPL* cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);

  cspace_i = factory.MakeCSpaceDecoratorNecessarySufficient(cspace_i, cspace_klampt_o);
  cspace_i->print();

  //if(level==0){
  //  cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);
  //}else if(level==1){
  //  std::vector<Config> path_constraint = node->content.path;
  //  cspace_i = factory.MakeGeometricCSpacePathConstraintRollInvariance(robot_inner, cspace_klampt_i, path_constraint);

  //##############################################################################
  //Compute roadmap for current cspace
  //##############################################################################
  StrategyGeometric strategy;
  StrategyOutput output(cspace_i);
  strategy.plan(input->GetStrategyInput(), cspace_i, output);

  std::vector<PathSpace*> decomposedspace;
  if(!output.success) return decomposedspace;
  //##############################################################################
  //compute vantage paths of each onetopic cover
  //##############################################################################

  OnetopicPathSpaceModifier onetopic_pathspace = OnetopicPathSpaceModifier(output.GetPlannerDataPtr(), cspace_i);
  std::vector<std::vector<Config>> vantage_paths = onetopic_pathspace.GetShortestPathForEachCover();

  std::vector<Config> all_vertices = onetopic_pathspace.GetAllVertices();
  std::vector<std::pair<Config,Config>> all_edges = onetopic_pathspace.GetAllEdges();
  std::vector<std::vector<Config>> all_paths = onetopic_pathspace.GetAllPaths();

  std::vector<std::vector<Config>> cover_vertices = onetopic_pathspace.GetCoverVertices();
  std::vector<std::vector<std::pair<Config,Config>>> cover_edges = onetopic_pathspace.GetCoverEdges();
  std::vector<std::vector<std::vector<Config>>> cover_paths = onetopic_pathspace.GetCoverPaths();

  std::cout << "paths: " << all_paths.size() << std::endl;
  std::cout << "paths: " << vantage_paths.size() << std::endl;

  PathSpaceInput *next = input->GetNextLayer();

  //##############################################################################
  PathSpace *p0 = new PathSpaceAtomic(world, next);
  Config qi = input->q_init;
  Config qg = input->q_goal;
  std::vector<Config> initpath;
  initpath.push_back(qi);
  initpath.push_back(qg);
  p0->SetShortestPath( initpath );
  p0->SetVertices(all_vertices);
  p0->SetEdges(all_edges);
  p0->SetPaths(all_paths);
  p0->SetRoadmap(output.GetRoadmap());

  decomposedspace.push_back(p0);

  for(uint k = 0; k < vantage_paths.size(); k++){
    PathSpace *pk = new PathSpaceAtomic(world, next);
    pk->SetShortestPath( vantage_paths.at(k) );
    pk->SetVertices(cover_vertices.at(k));
    pk->SetEdges(cover_edges.at(k));
    std::cout << "cover paths[" << k << "] " << cover_paths.at(k).size() << std::endl;
    pk->SetPaths(cover_paths.at(k));
    decomposedspace.push_back(pk);
  }

  //##############################################################################

  return decomposedspace;
}


void PathSpaceOnetopicCover::DrawGL(GUIState&){
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi_in = input->q_init;
  const Config qg_in = input->q_goal;

  GLColor lightGreen(0.2,0.9,0.2,0.5);
  GLColor lightRed(0.9,0.2,0.2,0.5);
  GLColor magenta(0.9,0.1,0.9,0.5);

  GLDraw::drawRobotAtConfig(robot, qi_in, lightGreen);
  GLDraw::drawRobotAtConfig(robot, qg_in, lightRed);
}

