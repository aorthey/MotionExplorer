#include "cspace_sentinel.h"
#include "planner/planner_ompl_humanoid.h"

MotionPlannerOMPLHumanoid::MotionPlannerOMPLHumanoid(RobotWorld *world):
  MotionPlannerOMPL(world)
{
}

void HumanoidPropagatorIrreducible::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
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
  uSE3(0) = 0;//ucontrol[0]; //include torsion?
  uSE3(1) = 0;//ucontrol[1];
  uSE3(2) = ucontrol[2];
  uSE3(3) = ucontrol[3];
  uSE3(4) = ucontrol[4];
  uSE3(5) = 0;

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

  uint Nduration = N+6;
  double dt = ucontrol[Nduration];
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

  for(int i = 0; i < N; i++){
    qend[i+6] = x0[i+6] + dt*ucontrol[i+6];
  }

  qend[58] = qend[12];
  qend[59] = qend[11];
  qend[60] = qend[10];
  qend[61] = qend[9];
  qend[62] = qend[8];
  qend[63] = qend[7];
  qend[64] = qend[6];


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
MotionPlannerOMPLHumanoidValidityChecker::MotionPlannerOMPLHumanoidValidityChecker(const ob::SpaceInformationPtr &si, Robot *robot, CSpace* space):
  MotionPlannerOMPLValidityChecker(si,space),robot(robot)
{
}

bool MotionPlannerOMPLHumanoidValidityChecker::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();

  Config q = OMPLStateToConfig(state, ssp);

  robot->UpdateConfig(q);

  Vector3 lfoot(q[0],q[1],q[2]);
  Matrix3 R;
  R.setRotateZ(q[3]);
  Vector3 c = robot->GetCOM() - lfoot;

  Vector3 ex(1,0,0),ey(0,1,0),cx,cy;
  R.mul(ex,cx);
  R.mul(ey,cy);

  double ctx = dot(c,cx);
  double cty = dot(c,cy);

  double dx = sqrt((ctx)*(ctx));
  double dy = sqrt((cty)*(cty));

  if(dx>0.08) return false;

  //if(!robot->InJointLimits(q)) {
    //return false;
  //}
  //return _space->CheckCollisionFree();

  //PropertyMap pmap;
  //_space->Properties(pmap);

  return _space->IsFeasible(q) && si_->satisfiesBounds(state);
}

void PostRunEventHumanoid(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run, GeometricCSpaceOMPL *cspace)
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
    util::PrintCurrentTime();
    const ob::PathPtr &pp = pdef->getSolutionPath();
    oc::PathControl path_control = static_cast<oc::PathControl&>(*pp);
    og::PathGeometric path = path_control.asGeometric();

    //og::PathSimplifier shortcutter(si);
    //shortcutter.shortcutPath(path);

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace->getPtr());
      keyframes.push_back(cur);
    }
    std::string sfile = "humanoid_wall_full_"+std::to_string(pid)+".xml";
    std::cout << "Saving keyframes"<< std::endl;
    Save(keyframes, sfile.c_str());
  }else{
    std::cout << "Run " << pid << " no solution" << std::endl;

  }
  pid++;

}


bool MotionPlannerOMPLHumanoid::solve(Config &p_init, Config &p_goal)
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
  std::cout << "KCSPACE:" << std::endl <<  pmap << std::endl;

  //std::cout << "Init: " << (kcspace.IsFeasible(p_init)?"yes":"no") << std::endl;
  //std::cout << "Goal: " << (kcspace.IsFeasible(p_goal)?"yes":"no") << std::endl;
  //###########################################################################
  // Config init,goal to OMPL start/goal
  //###########################################################################

  GeometricCSpaceOMPL cspace(robot);

  ob::ScopedState<> start = ConfigToOMPLState(p_init, cspace.getPtr());
  ob::ScopedState<> goal  = ConfigToOMPLState(p_goal, cspace.getPtr());
  std::cout << start << std::endl;
  std::cout << goal << std::endl;
  //###########################################################################
  // Kinodynamic planner
  //###########################################################################
  //KINODYNAMIC PLANNERS
  double kappa_curvature = 2.66;

  uint NdimControl = robot->q.size();

  auto control_cspace(std::make_shared<oc::RealVectorControlSpace>(cspace.getPtr(), NdimControl+1));
  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(-1);
  cbounds.setHigh(1);

  uint effectiveControlDim = 0;
  for(int i = 0; i < NdimControl; i++){
    double qmin = robot->qMin(i);
    double qmax = robot->qMax(i);
    double d = sqrtf((qmin-qmax)*(qmin-qmax));
    if(d<1e-8){
      //remove zero-measure dimensions for control
      cbounds.setLow(i,0);
      cbounds.setHigh(i,0);
    }else{
      effectiveControlDim++;
    }
  }
  //for(int i = 0; i < 6; i++){
    //cbounds.setLow(i,0);
    //cbounds.setHigh(i,0);
  //}
  //cbounds.setLow(2,-kappa_curvature);
  //cbounds.setHigh(2,kappa_curvature);


  for(int i = 36; i < 42; i++){
    cbounds.setLow(i,0);cbounds.setHigh(i,0);
  }
  for(int i = 50; i < 56; i++){
    cbounds.setLow(i,0);cbounds.setHigh(i,0);
  }
  // Link[36] LHAND_LINK0 mass 0.0632782
  // Link[37] LHAND_LINK1 mass 0.0775929
  // Link[38] LHAND_LINK2 mass 0.205701
  // Link[39] LHAND_LINK3 mass 0.0745618
  // Link[40] LHAND_LINK4 mass 0.0710285
  // Link[41] LeftHandForceSensor mass 0.0001
  // Link[42] l_gripper mass 0.0001
  

  //   Link[50] RHAND_LINK0 mass 0.0632782
  //   Link[51] RHAND_LINK1 mass 0.0775929
  //   Link[52] RHAND_LINK2 mass 0.205701
  //   Link[53] RHAND_LINK3 mass 0.0745618
  //   Link[54] RHAND_LINK4 mass 0.0710285
  //   Link[55] RightHandForceSensor mass 0.0001
  //   Link[56] r_gripper mass 0.0001


  cbounds.setLow(NdimControl,0.01);//propagation step size
  cbounds.setHigh(NdimControl,0.05);

  control_cspace->setBounds(cbounds);

  oc::SimpleSetup ss(control_cspace);
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  auto cpropagate(std::make_shared<HumanoidPropagatorIrreducible>(si, &kcspace));

  //###########################################################################
  // choose planner
  //###########################################################################
  //const oc::SpaceInformationPtr si = ss.getSpaceInformation();
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::RRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SST>(si);
  ob::PlannerPtr ompl_planner = std::make_shared<oc::PDST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::KPIECE1>(si);

  //###########################################################################
  // setup and projection
  //###########################################################################

  ss.setStateValidityChecker(std::make_shared<MotionPlannerOMPLHumanoidValidityChecker>(si, robot, &kcspace));
  ss.setStatePropagator(cpropagate);

  double epsilon = 1;

  ss.setStartAndGoalStates(start, goal, epsilon);
  ss.setup();
  ss.setPlanner(ompl_planner);
  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0rHumanoid(ss.getStateSpace())));

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
  double duration = 3600*24;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(duration) );

  //###########################################################################
  // benchmark instead
  //###########################################################################
  ot::Benchmark benchmark(ss, "BenchmarkHumanoidWall");
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
  //benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
  benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));

  ot::Benchmark::Request req;
  req.maxTime = duration;
  req.maxMem = 10000.0;
  req.runCount = 10;
  req.displayProgress = true;

  benchmark.setPostRunEvent(std::bind(&PostRunEventHumanoid, std::placeholders::_1, std::placeholders::_2, &cspace));

  benchmark.benchmark(req);
  benchmark.saveResultsToFile();

  std::string file = "ompl_humanoid_benchmark_wall";
  std::string res = file+".log";
  benchmark.saveResultsToFile(res.c_str());

  std::string cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
  std::system(cmd.c_str());
  cmd = "cp "+file+".db"+" ../data/benchmarks/";
  std::system(cmd.c_str());

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

    og::PathSimplifier shortcutter(si);
    shortcutter.shortcutPath(path);

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

    std::string sfile = util::GetCurrentTimeString()+".xml";

    std::cout << "Saving keyframes"<< std::endl;
    ::Save(_keyframes, sfile.c_str());
  }else{
    std::cout << "No solution found" << std::endl;
  }

  return solved;
}

