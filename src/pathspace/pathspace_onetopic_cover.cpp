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

  SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
  CSpaceOMPL* cspace_i;

  cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);
  cspace_i->print();

  //if(level==0){
  //  cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(robot_inner, cspace_klampt_i);
  //}else if(level==1){
  //  std::vector<Config> path_constraint = node->content.path;
  //  cspace_i = factory.MakeGeometricCSpacePathConstraintRollInvariance(robot_inner, cspace_klampt_i, path_constraint);

  StrategyGeometric strategy;
  StrategyOutput output;
  strategy.plan(input->GetStrategyInput(), cspace_i, output);

  std::vector<PathSpace*> decomposedspace;

  if(!output.success) return decomposedspace;

  OnetopicPathSpaceModifier onetopic_pathspace = OnetopicPathSpaceModifier(*output.pd, cspace_i);
  std::vector<std::vector<Config>> vantage_paths = onetopic_pathspace.GetConfigPaths();
  PathSpaceInput *next = input->GetNextLayer();

  std::cout << next->type << std::endl;
  for(uint k = 0; k < vantage_paths.size(); k++){
    PathSpace *pk = new PathSpaceAtomic(world, next);
    pk->SetShortestPath( vantage_paths.at(k) );
    decomposedspace.push_back(pk);
    //output.paths.push_back( vantage_paths.at(k) );
  }

  //exit(0);

  //for(uint k = 0; k < output.paths.size(); k++){
  //  PathSpace *pk = new PathSpaceAtomic(world, input);
  //  pk->SetShortestPath( output.paths.at(k) );
  //  decomposedspace.push_back(pk);
  //}
  return decomposedspace;
}


void PathSpaceOnetopicCover::DrawGL(const GUIState&){
  uint ridx = input->robot_idx;
  Robot* robot = world->robots[ridx];
  const Config qi_in = input->q_init;
  const Config qg_in = input->q_goal;

  GLColor lightGreen(0.2,0.9,0.2,0.5);
  GLColor lightRed(0.9,0.2,0.2,0.5);
  GLColor magenta(0.9,0.1,0.9,0.5);

  GLDraw::drawRobotAtConfig(robot, qi_in, lightGreen);
  GLDraw::drawRobotAtConfig(robot, qg_in, lightRed);

  if(vantage_path.size()>0){
    //GLDraw::drawPath(vantage_path, magenta, 20);
    //const SweptVolume& sv = GetSweptVolume(robot);
    //GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  }
}
