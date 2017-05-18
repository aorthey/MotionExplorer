#include "cspace_sentinel.h"
#include "planner/planner_ompl_irreducible.h"

MotionPlannerOMPLIrreducible::MotionPlannerOMPLIrreducible(RobotWorld *world):
  MotionPlannerOMPL(world)
{
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj2(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}

void SentinelPropagatorIrreducible::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{

  //###########################################################################
  // OMPL to Config control
  //###########################################################################
  const ob::StateSpacePtr s = si_->getStateSpace();
  Config x0 = OMPLStateToConfig(state, s);

  const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  Config uSE3;
  uSE3.resize(6);
  uSE3.setZero();
  uSE3(0) = 0.0;//ucontrol[0]; //include torsion?
  uSE3(1) = ucontrol[1];
  uSE3(2) = ucontrol[2];
  uSE3(3) = 1.0;
  uSE3(4) = 0.0;
  uSE3(5) = 0.0;

  uint N = s->getDimension() - 6;
  //###########################################################################
  // Forward Simulate
  //###########################################################################
  //double roll = u(0);
  //double pitch = u(1);
  //double yaw = u(2);
  //Math3D::EulerAngleRotation Reuler(roll, pitch, yaw);
  //Math3D::Matrix3 R;
  //Reuler.getMatrixXYZ(R);

  //std::vector<Config> path;
  //cspace_->Simulate(x0, u, path);
  //Config qend = path.back();

  //new
  double dt = ucontrol[3];
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }

  Matrix4 x0_SE3 = cspace_->StateToSE3(x0);
  Matrix4 dp0 = cspace_->SE3Derivative(uSE3);
  Matrix4 x1_SE3 = cspace_->ForwardSimulate(x0_SE3,dp0,dt);

  State x1(x0);
  cspace_->SE3ToState(x1, x1_SE3);

  Config qend = x1;

  //###########################################################################
  // Config to OMPL
  //###########################################################################

  //std::cout << x0 << std::endl;
  //std::cout << qend << std::endl;

  ob::ScopedState<> ssr = ConfigToOMPLState(qend, s);

  ob::SE3StateSpace::StateType *ssrSE3 = ssr->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *ssrSO3 = &ssrSE3->rotation();
  ob::RealVectorStateSpace::StateType *ssrRn = ssr->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

  ob::SE3StateSpace::StateType *resultSE3 = result->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *resultSO3 = &resultSE3->rotation();
  ob::RealVectorStateSpace::StateType *resultRn = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

  resultSE3->setXYZ(ssrSE3->getX(),ssrSE3->getY(),ssrSE3->getZ());
  resultSO3->x = ssrSO3->x;
  resultSO3->y = ssrSO3->y;
  resultSO3->z = ssrSO3->z;
  resultSO3->w = ssrSO3->w;

  //###########################################################################
  // R^N Control
  //###########################################################################
  for(int i = 0; i < N; i++){
    resultRn->values[i] = ssrRn->values[i];
  }

}

void PostRunEventIrreducible(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run, GeometricCSpaceOMPL *cspace)
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

  if(solved){
    std::cout << "Found Solution at run " << pid << std::endl;
    const ob::PathPtr &pp = pdef->getSolutionPath();
    oc::PathControl path_control = static_cast<oc::PathControl&>(*pp);
    og::PathGeometric path = path_control.asGeometric();

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace->getPtr());
      keyframes.push_back(cur);
    }
    std::string sfile = "ompl"+std::to_string(pid)+".xml";
    std::cout << "Saving keyframes"<< std::endl;
    Save(keyframes, sfile.c_str());
  }else{
    std::cout << "Run " << pid << " no solution" << std::endl;

  }
  pid++;

}


bool MotionPlannerOMPLIrreducible::solve(Config &p_init, Config &p_goal)
{
  this->_p_init = p_init;
  this->_p_goal = p_goal;
  Robot *robot = _world->robots[_irobot];
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
  SingleRobotCSpace geometric_cspace = SingleRobotCSpace(*_world,_irobot,&worldsettings);
  if(!IsFeasible( robot, geometric_cspace, _p_init)) return false;
  if(!IsFeasible( robot, geometric_cspace, _p_goal)) return false;

  KinodynamicCSpaceSentinelAdaptor kcspace(&geometric_cspace);
  PropertyMap pmap;
  kcspace.Properties(pmap);
  std::cout << pmap << std::endl;

  //###########################################################################
  // Config init,goal to OMPL start/goal
  //###########################################################################

  GeometricCSpaceOMPL cspace(robot);

  ob::ScopedState<> start = ConfigToOMPLState(p_init, cspace.getPtr());
  ob::ScopedState<> goal  = ConfigToOMPLState(p_goal, cspace.getPtr());
  std::cout << start << std::endl;
  std::cout << goal << std::endl;


  ////###########################################################################
  //// Geometric planning (tested, works)
  ////###########################################################################
  //bool geometric = false;
  //if(geometric){

  //  og::SimpleSetup ss(cspace.getPtr());
  //  const ob::SpaceInformationPtr si = ss.getSpaceInformation();

  //  //GEOMETRIC PLANNERS
  //  //ob::PlannerPtr ompl_planner = std::make_shared<og::RRTConnect>(si);
  //  ob::PlannerPtr ompl_planner = std::make_shared<og::RRT>(si);
  //  //ob::PlannerPtr ompl_planner = std::make_shared<og::RRTstar>(si);
  //  //ob::PlannerPtr ompl_planner = std::make_shared<og::RRTsharp>(si);
  //  //ob::PlannerPtr ompl_planner = std::make_shared<og::LazyRRT>(si);

  //  ss.setPlanner(ompl_planner);
  //  //ss.setPlanner(std::make_shared<og::RRTConnect>(si));

  //  si->setStateValidityChecker(std::make_shared<MotionPlannerOMPLValidityChecker>(si, &kcspace));

  //  ss.setStartAndGoalStates(start, goal);
  //  ss.setup();
  //  ob::PlannerStatus solved = ss.solve(10.0);
  //  if (solved)
  //  {
  //    std::cout << "Found solution:" << std::endl;
  //    ss.simplifySolution();
  //    og::PathGeometric path = ss.getSolutionPath();
  //    path.interpolate();
  //    std::cout << path.length() << std::endl;
  //    std::cout << path.getStateCount() << std::endl;

  //    vector<Config> keyframes;
  //    for(int i = 0; i < path.getStateCount(); i++)
  //    {
  //      ob::State *state = path.getState(i);
  //      Config cur = OMPLStateToConfig(state, cspace.getPtr());
  //      _keyframes.push_back(cur);
  //    }
  //    std::cout << std::string(80, '-') << std::endl;
  //  }else{
  //    std::cout << "No solution found" << std::endl;
  //  }
  //}else{
  //###########################################################################
  // Kinodynamic planner
  //###########################################################################
  //KINODYNAMIC PLANNERS
  //double kappa_curvature = 2.2;
  double kappa_curvature = 1.57;
  uint NdimControl = 3;

  //TODO: this does not check if index is out of bounds -> 
  auto control_cspace(std::make_shared<oc::RealVectorControlSpace>(cspace.getPtr(), NdimControl+1));
  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(-kappa_curvature);
  cbounds.setHigh(kappa_curvature);

  cbounds.setLow(NdimControl,0.01);//propagation step size
  cbounds.setHigh(NdimControl,0.10);

  control_cspace->setBounds(cbounds);

  oc::SimpleSetup ss(control_cspace);
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  auto cpropagate(std::make_shared<SentinelPropagatorIrreducible>(si, &kcspace));

  //###########################################################################
  // choose planner
  //###########################################################################
  //const oc::SpaceInformationPtr si = ss.getSpaceInformation();
  ob::PlannerPtr ompl_planner = std::make_shared<oc::RRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::PDST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::KPIECE1>(si);
  //
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::Syclop>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SyclopRRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SyclopEST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::LTLPlanner>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::EST>(si);

  //###########################################################################
  // setup and projection
  //###########################################################################

  ss.setStateValidityChecker(std::make_shared<MotionPlannerOMPLValidityChecker>(si, &kcspace));
  ss.setStatePropagator(cpropagate);

  double epsilon = 0.5;

  ss.setStartAndGoalStates(start, goal, epsilon);
  ss.setup();
  ss.setPlanner(ompl_planner);
  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLengthObj2(si) );

  //###########################################################################
  // debug
  //###########################################################################

  //testRotationConversion();
  //test();
  //testSE3(kcspace);

  //###########################################################################
  // solve
  //
  // termination condition: 
  //    reached duration or found solution in epsilon-neighborhood
  //###########################################################################
  bool solved = false;
  double solution_time = dInf;
  double duration = 1200.0;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(duration) );

  //###########################################################################
  // benchmark instead
  //###########################################################################
  // ot::Benchmark benchmark(ss, "BenchmarkSnakeTurbineIrreducible");
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));

  // ot::Benchmark::Request req;
  // req.maxTime = duration;
  // req.maxMem = 10000.0;
  // req.runCount = 100;
  // req.displayProgress = true;

  // benchmark.setPostRunEvent(std::bind(&PostRunEventIrreducible, std::placeholders::_1, std::placeholders::_2, &cspace));

  // benchmark.benchmark(req);
  // benchmark.saveResultsToFile();

  // std::string file = "ompl_irreducible_benchmark";
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
  //std::cout << "Edges   : " << pd.numEdges() << std::endl;
  //std::cout << "Vertices: " << pd.numVertices() << std::endl;

  SerializeTree(pd);
  //SerializeTreeRandomlyCullPoints(_stree, 2000);

  //###########################################################################
  // extract solution path if solved
  //###########################################################################

  solved = ss.haveSolutionPath();
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

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace.getPtr());
      _keyframes.push_back(cur);
    }
    ob::State *obgoal = path.getState(path.getStateCount()-1);
    Config plannergoal = OMPLStateToConfig(obgoal, cspace.getPtr());

    uint istep = max(int(path.getStateCount()/10.0),1);
    for(int i = 0; i < path.getStateCount(); i+=istep)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace.getPtr());
      std::cout << i << "/" << path.getStateCount() <<  cur << std::endl;
    }
    std::cout << std::string(80, '-') << std::endl;
  }else{
    std::cout << "No solution found" << std::endl;
  }

  return solved;
}

