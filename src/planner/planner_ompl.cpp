#include "planner_ompl.h"

#include "cspace_sentinel.h"
#include "liegroupintegrator.h"


void MotionPlannerOMPL::SerializeTree(ob::PlannerData &pd)
{
  //pd.decoupleFromPlanner();
  std::cout << "serializing tree with " << pd.numVertices() << " vertices" << std::endl;
  std::cout << "                  and " << pd.numEdges() << " edges" << std::endl;
  pd.toBoostGraph();

  _stree.clear();
  for(uint i = 0; i < pd.numVertices(); i++){
    ob::PlannerDataVertex v = pd.getVertex(i);
    const ob::State* s = v.getState();
    const ob::SE3StateSpace::StateType *sSE3 = s->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    double x = sSE3->getX();
    double y = sSE3->getY();
    double z = sSE3->getZ();
    SerializedTreeNode snode;
    Config q;q.resize(6);q.setZero();
    q(0)=x;
    q(1)=y;
    q(2)=z;
    snode.position = q;

    //std::vector<uint> edgeList;
    //uint Nedges = pd.getIncomingEdges (i, edgeList);
     
    std::vector<uint> edgeList;
    pd.getEdges(i, edgeList);
    //std::cout << "Node " << i << " has " << Nedges << " edges" << std::endl;
    for(int j = 0; j < edgeList.size(); j++){
      ob::PlannerDataVertex w = pd.getVertex(edgeList.at(j));
      const ob::State* sw = w.getState();
      const ob::SE3StateSpace::StateType *swSE3 = sw->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
      double xw = swSE3->getX();
      double yw = swSE3->getY();
      double zw = swSE3->getZ();

      Vector3 dvw( xw-x, yw-y, zw-z);
      snode.directions.push_back(dvw);
    }



    _stree.push_back(snode);
    
  }
}

MotionPlannerOMPL::MotionPlannerOMPL(RobotWorld *world):
  MotionPlanner(world)
{
}

void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run, GeometricCSpaceOMPL *cspace)
{
  static uint pid = 0;

  //run["some extra property name INTEGER"] = "some value";
  // The format of added data is string key, string value pairs,
  // with the convention that the last word in string key is one of
  // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
  // when the log file is processed and saved as a database).
  // The values are always converted to string.
  ob::SpaceInformationPtr si = planner->getSpaceInformation();
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
  bool solved = pdef->hasExactSolution();

  double dg = pdef->getSolutionDifference();
  std::cout << dg << std::endl;

  if(solved){
    std::cout << "Found Solution at run " << pid << std::endl;
    const ob::PathPtr &pp = pdef->getSolutionPath();
    oc::PathControl path_control = static_cast<oc::PathControl&>(*pp);
    og::PathGeometric path = path_control.asGeometric();

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace->SpacePtr());
      keyframes.push_back(cur);
    }
    std::string sfile = "ompl_"+std::to_string(pid)+".xml";
    std::cout << "Saving keyframes"<< std::endl;
    Save(keyframes, sfile.c_str());
  }else{
    std::cout << "Run " << pid << " no solution" << std::endl;

  }
  pid++;

}

bool MotionPlannerOMPL::solve(Config &p_init, Config &p_goal)
{
  this->_p_init = p_init;
  this->_p_goal = p_goal;
  robot->UpdateConfig(_p_init);

  this->_world->InitCollisions();

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Motion Planner:" << this->getName() << std::endl;
  std::cout << "p_init =" << p_init << std::endl;
  std::cout << "p_goal =" << p_goal << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  //###########################################################################
  // Setup Klampt CSpace
  //###########################################################################

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*_world);

  SingleRobotCSpace kcspace = SingleRobotCSpace(*_world,_irobot,&worldsettings);
  if(!IsFeasible( robot, kcspace, _p_goal)) return false;
  if(!IsFeasible( robot, kcspace, _p_init)) return false;

  //
  //GeometricCSpaceOMPL: Space = Configuration manifold; control space = tangent
  //space of configuration manifold, i.e. control happens in velocity space
  //
  //KinodynamicCSpaceOMPL: Space = tangent bundle of configuration manifold;
  //control space = tangent space of tangent bundle, i.e. control happens in
  //acceleration space, i.e. we can control torques of revolute joints, forces
  //of prismatic joints, and any additional booster/thruster which act directly
  //on the se(3) component
  //

  CSpaceFactory factory;
  //GeometricCSpaceOMPL cspace(robot, &kcspace);
  //KinodynamicCSpaceOMPL cspace(robot, &kcspace);

  //GeometricCSpaceOMPL* cspace = factory.MakeGeometricCSpace(robot, &kcspace);
  KinodynamicCSpaceOMPL* cspace = factory.MakeKinodynamicCSpace(robot, &kcspace);

  //###########################################################################
  // Config init,goal to OMPL start/goal
  //###########################################################################

  ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
  std::cout << start << std::endl;
  std::cout << goal << std::endl;
  //exit(0);

  oc::SimpleSetup ss(cspace->ControlSpacePtr());//const ControlSpacePtr
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  ss.setStateValidityChecker(cspace->StateValidityCheckerPtr(si));
  ss.setStatePropagator(cspace->StatePropagatorPtr(si));

  //###########################################################################
  // choose planner
  //###########################################################################
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::RRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::PDST>(si);
  ob::PlannerPtr ompl_planner = std::make_shared<oc::KPIECE1>(si);

  //###########################################################################
  // setup and projection
  //###########################################################################
  double epsilon = 1.0;

  ss.setStartAndGoalStates(start, goal, epsilon);
  ss.setup();
  ss.setPlanner(ompl_planner);
  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

  //###########################################################################
  // solve
  //
  // termination condition: 
  //    reached duration or found solution in epsilon-neighborhood
  //###########################################################################
  bool solved = false;
  double solution_time = dInf;
  double duration = 120.0;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(duration) );

  //###########################################################################
  // benchmark instead
  //###########################################################################
  // ot::Benchmark benchmark(ss, "BenchmarkSnakeTurbine");
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));

  // ot::Benchmark::Request req;
  // req.maxTime = duration;
  // req.maxMem = 10000.0;
  // req.runCount = 100;
  // req.displayProgress = true;

  // benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2, &cspace));
  
  // benchmark.benchmark(req);
  // benchmark.saveResultsToFile();

  // std::string file = "ompl_benchmark";
  // std::string res = file+".log";
  // benchmark.saveResultsToFile(res.c_str());

  // std::string cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
  // std::system(cmd.c_str());
  // cmd = "cp "+file+".db"+" ../data/benchmarks/";
  // std::system(cmd.c_str());

  //###########################################################################
  // solve
  //###########################################################################

  ob::PlannerStatus status = ss.solve(ptc);
  solved = ss.haveExactSolutionPath();

  //###########################################################################
  // extract roadmap
  //###########################################################################

  oc::PlannerData pd(si);
  ss.getPlannerData(pd);
  SerializeTree(pd);

  //###########################################################################
  // extract solution path if solved
  //###########################################################################

  solved = ss.haveSolutionPath();
  //return approximate solution
  if (solved)
  {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Found solution:" << std::endl;
    std::cout << " exact solution       : " << (pdef->hasExactSolution()? "Yes":"No")<< std::endl;
    std::cout << " approximate solution : " << (pdef->hasApproximateSolution()? "Yes":"No")<< std::endl;

    double dg = pdef->getSolutionDifference();
    std::cout << " solution difference  : " << dg << std::endl;
    oc::PathControl path_control = ss.getSolutionPath();
    og::PathGeometric path = path_control.asGeometric();
    std::cout << "Path Length     : " << path.length() << std::endl;
    std::cout << "Path Milestones : " << path.getStateCount() << std::endl;

    //og::PathSimplifier shortcutter(si);
    //shortcutter.shortcutPath(path);

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace->SpacePtr());
      _keyframes.push_back(cur);
    }
    ob::State *obgoal = path.getState(path.getStateCount()-1);
    Config plannergoal = OMPLStateToConfig(obgoal, cspace->SpacePtr());

    uint istep = max(int(path.getStateCount()/10.0),1);
    for(int i = 0; i < path.getStateCount(); i+=istep)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace->SpacePtr());
      std::cout << i << "/" << path.getStateCount() <<  cur << std::endl;
    }
    std::cout << std::string(80, '-') << std::endl;
  }else{
    std::cout << "No solution found" << std::endl;
  }

  return solved;
}

