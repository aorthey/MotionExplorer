#include "cspace_sentinel.h"
#include "planner/planner_ompl_irreducible.h"

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

  Config u;
  u.resize(7);
  u.setZero();
  u(0) = 0.0;//ucontrol[0]; //include torsion?
  u(1) = ucontrol[0];
  u(2) = ucontrol[1];
  u(3) = 1.0;
  u(6) = ucontrol[3];//time


  //###########################################################################
  // Forward Simulate
  //###########################################################################
  double roll = u(0);
  double pitch = u(1);
  double yaw = u(2);
  Math3D::EulerAngleRotation Reuler(roll, pitch, yaw);
  Math3D::Matrix3 R;
  Reuler.getMatrixXYZ(R);

  //Matrix4 tmp = MatrixExponential(dp0*h);
  //Matrix4 pnext = p0*tmp;

  std::vector<Config> path;
  cspace_->Simulate(x0, u, path);

  Config qend = path.back();

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
  uint N = s->getDimension() - 6;
  for(int i = 0; i < N; i++){
    resultRn->values[i] = ssrRn->values[i];
  }

}


bool MotionPlannerOMPLIrreducible::solve(Config &p_init, Config &p_goal)
{
  this->_p_init = p_init;
  this->_p_goal = p_goal;
  Robot *robot = _world->robots[_irobot];
  robot->UpdateConfig(_p_init);

  util::SetSimulatedRobot(robot,*_sim,_p_init);//TODO: outsource sim
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
  uint NdimControl = 3;
  auto control_cspace(std::make_shared<oc::RealVectorControlSpace>(cspace.getPtr(), NdimControl+1));
  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(-1);
  cbounds.setHigh(1);

  cbounds.setLow(3,0.02);//propagation step size
  cbounds.setHigh(3,0.15);


  control_cspace->setBounds(cbounds);

  oc::SimpleSetup ss(control_cspace);
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  auto cpropagate(std::make_shared<SentinelPropagator>(si, &kcspace));

  //###########################################################################
  // choose planner
  //###########################################################################
  //const oc::SpaceInformationPtr si = ss.getSpaceInformation();
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::RRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SST>(si);
  ob::PlannerPtr ompl_planner = std::make_shared<oc::PDST>(si);
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
  double duration = 3600.0;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(duration) );


  //###########################################################################
  // benchmark instead
  //###########################################################################
  //ot::Benchmark benchmark(ss, "IrreducibleBenchmarkPipes");
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));

  //ot::Benchmark::Request req;
  //req.maxTime = duration;
  //req.maxMem = 10000.0;
  //req.runCount = 100;
  //req.displayProgress = true;
  //benchmark.benchmark(req);
  //// This will generate a file of the form ompl_host_time.log
  //benchmark.saveResultsToFile();


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
  SerializeTreeRandomlyCullPoints(_stree, 2000);

  //###########################################################################
  // extract solution path if solved
  //###########################################################################

  if (solved)
  {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Found solution:" << std::endl;
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

    //double dgoal = kcspace.Distance(plannergoal, p_goal);
    //std::cout << "Distance to goal: " << dgoal << std::endl;

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

