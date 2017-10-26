#include "pathspace/pathspace_linear_hierarchy.h"
#include "pathspace_atomic.h"
#include "planner/strategy/strategy_geometric.h"
#include "planner/strategy/strategy_roadmap.h"
#include "planner/strategy/strategy_input_edgegoal.h"
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
  StrategyInput strategy_input = input->GetStrategyInput();
  strategy_input.cspace = cspace;

  strategy.plan(strategy_input, output);

  //###########################################################################
  // Extract from the planner the roadmap, and decompose the roadmap into a
  // roadmap containing only sufficient vertices (sufficient_roadmap), and a
  // roadmap which contains only necessary but not sufficient vertices
  // (necessary_roadmap). 
  //###########################################################################

  Roadmap sufficient_roadmap;
  sufficient_roadmap.CreateFromPlannerDataOnlySufficient(output.GetPlannerDataPtr(), cspace);
  sufficient_roadmap.cVertex = green;
  sufficient_roadmap.cEdge = green;

  Roadmap necessary_roadmap;
  necessary_roadmap.CreateFromPlannerDataOnlyNecessary(output.GetPlannerDataPtr(), cspace);
  necessary_roadmap.cVertex = magenta;
  necessary_roadmap.cEdge = magenta;

  RoadmapPtr roadmap = output.GetRoadmapPtr();
  roadmap->cVertex = green;
  roadmap->cEdge = green;

  //###########################################################################
  // make sure that the output path and the shortest path along plannerdata are
  // the same
  //###########################################################################
  // std::cout << std::string(80, '-') << std::endl;

  // std::vector<Config> opath = output.GetShortestPath();
  // for(uint k = 0; k < opath.size(); k++){
  //   std::cout << opath.at(k) << std::endl;
  // }

  // std::cout << std::string(80, '-') << std::endl;

  // std::vector<Config> shortest_path = roadmap->GetShortestPath();
  // for(uint k = 0; k < shortest_path.size(); k++){
  //   std::cout << shortest_path.at(k) << std::endl;
  // }

  // std::cout << std::string(80, '-') << std::endl;

  //###########################################################################
  std::vector<PathSpace*> decomposedspace;

  bool done = false;
  PathSpaceInput *level0 = input->GetNextLayer();
  PathSpaceInput *level1 = level0->GetNextLayer();
  while(!done){
  //  //(1) get shortest path from roadmap
    std::vector<Config> shortest_path = roadmap->GetShortestPath();

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "shortest path:"  << std::endl;
    for(uint k = 0; k < shortest_path.size(); k++){
      std::cout << shortest_path.at(k) << std::endl;
    }

    std::cout << std::string(80, '-') << std::endl;
    if(shortest_path.empty()){
      std::cout << "no more shortest paths found" << std::endl;
      decomposedspace.push_back( new PathSpaceAtomic(world, level1) );
      decomposedspace.back()->SetRoadmap( roadmap );
      break;
    }

    CSpaceFactory factory(level1->GetCSpaceInput());

    uint inner_index = level1->robot_inner_idx;
    uint outer_index = level1->robot_outer_idx;

    Robot* robot_inner = world->robots[inner_index];
    Robot* robot_outer = world->robots[outer_index];

    SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,inner_index,&worldsettings);
    SingleRobotCSpace* cspace_klampt_o = new SingleRobotCSpace(*world,outer_index,&worldsettings);

    uint last_index;
    bool failed_edge = false;
    Config qedge_start = level1->q_init;

    std::vector<Config> edge_path;
    for(uint k = 0; k < shortest_path.size()-1; k++){
      std::vector<Config> edge;
      edge.push_back(shortest_path.at(k));
      edge.push_back(shortest_path.at(k+1));
      CSpaceOMPL* cspace = factory.MakeGeometricCSpacePathConstraintSO3(robot_inner, cspace_klampt_i, edge);

      cspace->print();

      StrategyInput input_tmp = level1->GetStrategyInput();
      StrategyInputEdgeGoal input_level1(input_tmp);

      //project onto edge
      input_level1.q_init = qedge_start;
      input_level1.q_init(0) = shortest_path.at(k)(0);
      input_level1.q_init(1) = shortest_path.at(k)(1);
      input_level1.q_init(2) = shortest_path.at(k)(2);
      input_level1.q_goal = level1->q_goal;
      input_level1.q_goal(0) = shortest_path.at(k+1)(0);
      input_level1.q_goal(1) = shortest_path.at(k+1)(1);
      input_level1.q_goal(2) = shortest_path.at(k+1)(2);

      input_level1.name_algorithm = "ompl:rrt";
      input_level1.cspace = cspace;
      input_level1.max_planning_time = 0.05;

      StrategyGeometric strategy;
      StrategyOutput output_level1(cspace);

      ob::ScopedState<> ss = cspace->ConfigToOMPLState(input_level1.q_init);
      ob::State *s = ss.get();
      const ob::RealVectorStateSpace::StateType *qomplRnSpace = s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
      double d = qomplRnSpace->values[0];
      double epsilon = 1e-10;
      if( abs(d) > epsilon){
        std::cout << "error start state" << std::endl;
        std::cout << d << std::endl;
        std::cout << edge.at(0) << std::endl;
        std::cout << input_level1.q_init << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        std::cout << edge.at(1) << std::endl;
        std::cout << input_level1.q_goal << std::endl;
        exit(0);
      }

      //############################################################/
      ///DESIGN GOAL REGION WHICH HAS ONLY N-1 DIMENSIONS
      //############################################################/
      strategy.plan(input_level1, output_level1);

      last_index = k;
      if(output_level1.hasExactSolution()){
        std::vector<Config> edge_shortest_path = output_level1.GetShortestPath();
        for(uint j = 0; j < 6; j++){
          qedge_start(j) = edge_shortest_path.back()(j);
        }
        for(uint j = 6; j < level1->q_init.size(); j++){
          qedge_start(j) = level1->q_init(j);
        }
        for(uint j = 0; j < edge_shortest_path.size(); j++){
          Config q = level1->q_init;
          for(uint m = 0; m < 6; m++){
            q(m) = edge_shortest_path.at(j)(m);
          }
          edge_path.push_back(q);
          //edge_path.insert(edge_path.end(), edge_shortest_path.begin(),edge_shortest_path.end());
        }
      }else{
        //identify infeasible edges.
        std::cout << "Infeasible Edge" << std::endl;
        std::cout << shortest_path.at(k) << std::endl;
        std::cout << shortest_path.at(k+1) << std::endl;
        roadmap->removeInfeasibleEdgeAlongShortestPath(last_index);
        failed_edge = true;
        break;
      }
    }

    if(failed_edge){
      std::cout << "failed at edge: " << last_index << "/" << shortest_path.size() << std::endl;
    }else{
      std::cout << "path is success" << std::endl;
      decomposedspace.push_back( new PathSpaceAtomic(world, level1) );
      decomposedspace.back()->SetRoadmap( roadmap );
      decomposedspace.back()->SetShortestPath( edge_path );
      done=true;
    }
  }

  //###########################################################################
  //for each necessary vertex, create a new underlying cspace and create a
  //roadmap. this roadmap is then sampled until we find a feasible point. this
  //vertex is then added to the sufficient_roadmap, marking a feasible vertex.
  //The underlying roadmap should still grow, because there might be
  //disconnected components for each vertex
  //###########################################################################

  //decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
  //decomposedspace.back()->SetRoadmap( &necessary_roadmap );
  //decomposedspace.push_back( new PathSpaceAtomic(world, input->GetNextLayer()) );
  //decomposedspace.back()->SetRoadmap( sufficient_roadmap );
  return decomposedspace;

}
