#include "planner_ompl.h"

#include "cspace_sentinel.h"
#include "liegroupintegrator.h"

GeometricCSpaceOMPL::GeometricCSpaceOMPL(Robot *robot)
{
  //###########################################################################
  // Create OMPL state space
  //   Create an SE(3) x R^n state space
  //###########################################################################
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "[MotionPlanner] only supports robots with a configuration space equal to SE(3) x R^n" << std::endl;
    exit(0);
  }

  double N = robot->q.size()-6;
  std::cout << "[MotionPlanner] Robot CSpace: SE(3) x R^"<<N<<std::endl;

  ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
  ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));

  space_ = SE3 + Rn;
  ob::SE3StateSpace *cspaceSE3 = space_->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);
  ob::RealVectorStateSpace *cspaceRn = space_->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);

  ob::CompoundStateSpace *cspace = space_->as<ob::CompoundStateSpace>();

  //cspace->setSubspaceWeight(0,1);
  //cspace->setSubspaceWeight(1,0);

  //###########################################################################
  // Set bounds
  //###########################################################################
  std::vector<double> minimum, maximum;
  //PropertyMap props;
  //space->Properties(props);
  //props.getArray("minimum",minimum);
  //props.getArray("maximum",maximum);
  minimum = robot->qMin;
  maximum = robot->qMax;

  assert(minimum.size() == 6+N);
  assert(maximum.size() == 6+N);

  vector<double> lowSE3;
  lowSE3.push_back(minimum.at(0));
  lowSE3.push_back(minimum.at(1));
  lowSE3.push_back(minimum.at(2));
  vector<double> highSE3;
  highSE3.push_back(maximum.at(0));
  highSE3.push_back(maximum.at(1));
  highSE3.push_back(maximum.at(2));

  ob::RealVectorBounds boundsSE3(3);
  boundsSE3.low = lowSE3;
  boundsSE3.high = highSE3;
  cspaceSE3->setBounds(boundsSE3);

  vector<double> lowRn, highRn;
  for(int i = 0; i < N; i++){
    lowRn.push_back(minimum.at(i+6));
    highRn.push_back(maximum.at(i+6));
  }
  ob::RealVectorBounds boundsRn(N);

  //ompl does only accept dimensions with strictly positive measure, adding some epsilon space
  double epsilonSpacing=1e-8;
  for(int i = 0; i < N; i++){
    if(abs(lowRn.at(i)-highRn.at(i))<epsilonSpacing){
      highRn.at(i)+=epsilonSpacing;
    }
  }

  boundsRn.low = lowRn;
  boundsRn.high = highRn;
  cspaceRn->setBounds(boundsRn);

}

void MotionPlannerOMPL::SerializeTree(ob::PlannerData &pd)
{
  //pd.decoupleFromPlanner();
  std::cout << "serializing tree with " << pd.numVertices() << " vertices" << std::endl;
  std::cout << "                  and " << pd.numEdges() << " edges" << std::endl;
  pd.toBoostGraph();

  //ob::PlannerData::Graph::EIterator eiter;
  //for(eiter = graph.begin(); eiter!=graph.end(); ++eiter){
  //}

  //graph_traits < adjacency_list <> >::vertex_iterator i, end;
  //graph_traits < adjacency_list <> >::adjacency_iterator ai, a_end;
  //property_map < adjacency_list <>, vertex_index_t >::type
  //  index_map = get(vertex_index, g);

  //for (tie(i, end) = vertices(g); i != end; ++i) {
  //  std::cout << name[get(index_map, *i)];
  //  tie(ai, a_end) = adjacent_vertices(*i, g);
  //  if (ai == a_end)
  //    std::cout << " has no children";
  //  else
  //    std::cout << " is the parent of ";
  //  for (; ai != a_end; ++ai) {
  //    std::cout << name[get(index_map, *ai)];
  //    if (boost::next(ai) != a_end)
  //      std::cout << ", ";
  //  }
  //  std::cout << std::endl;
  //}

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

MotionPlannerOMPLValidityChecker::MotionPlannerOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace* space):
  ob::StateValidityChecker(si),_space(space)
{
}

bool MotionPlannerOMPLValidityChecker::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = OMPLStateToConfig(state, ssp);
  //Robot *robot = _space->robot;
  //if(!robot->InJointLimits(q)) {
    //return false;
  //}
  //return _space->CheckCollisionFree();

  //PropertyMap pmap;
  //_space->Properties(pmap);

  return _space->IsFeasible(q) && si_->satisfiesBounds(state);
}

MotionPlannerOMPL::MotionPlannerOMPL(RobotWorld *world):
  MotionPlanner(world)
{
}
// void MotionPlannerOMPL::testSE3(KinodynamicCSpaceSentinelAdaptor &cspace)
// {
//   std::cout << "Testing Klampt->SE3->Klampt conversion" << std::endl;

//   Srand(0);
//   for(int i = 0; i < 100; i++){
//     uint N = Rand(0,0);
//     ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
//     ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));
//     ob::StateSpacePtr stateSpace = SE3 + Rn;

//     Config q;
//     q.resize(6+N);
//     q.setZero();
//     q[0]=Rand(-3,3);
//     q[1]=Rand(-3,3);
//     q[2]=Rand(-3,3);
//     q[3] = Rand(-M_PI,M_PI);
//     q[4] = Rand(-M_PI/2,M_PI/2);
//     q[5] = Rand(-M_PI,M_PI);
//     for(int j = 0; j < N; j++){
//       q[j+6] = Rand(0,3);
//     }
//     Matrix4 x = cspace.StateToSE3(q);
//     Config qq(q);
//     qq.setZero();
//     cspace.SE3ToState(qq,x);

//     double epsilon = 1e-10;
//     if((q-qq).norm()>epsilon){
//       std::cout << "Klampt->SE3->Klampt Test "<<i<<" failed" << std::endl;
//       std::cout << std::string(80, '-') << std::endl;
//       std::cout << "Klampt Input  state: " << q << std::endl;
//       std::cout << "Klampt Output state: " << qq << std::endl;
//       std::cout << std::string(80, '-') << std::endl;
//       exit(0);
//     }
//   }

// }

// void MotionPlannerOMPL::test()
// {
//   std::cout << "Testing Klampt->OMPL->Klampt conversion" << std::endl;

//   Srand(0);
//   for(int i = 0; i < 100; i++){
//     uint N = Rand(0,0);
//     ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
//     ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));
//     ob::StateSpacePtr stateSpace = SE3 + Rn;

//     Config q;
//     q.resize(6+N);
//     q.setZero();
//     q[0]=Rand(-3,3);
//     q[1]=Rand(-3,3);
//     q[2]=Rand(-3,3);
//     q[3] = Rand(-M_PI,M_PI);
//     q[4] = Rand(-M_PI/2,M_PI/2);
//     q[5] = Rand(-M_PI,M_PI);
//     for(int j = 0; j < N; j++){
//       q[j+6] = Rand(0,3);
//     }
//     //std::cout << "testing " << i << " : " << std::endl;
//     test_conversion(q, stateSpace);
//   }


// }
//void checkYawPitchRoll(double y, double p, double r, double y2, double p2, double r2)
//{
//  double epsilon = 1e-10;
//  double dz = fabs(y-y2);
//  double dy = fabs(p-p2);
//  double dx = fabs(r-r2);
//  //std::cout << "Original state: " << y << "," << p << "," << r << std::endl;
//  //std::cout << "Convertd state: " << y2 << "," << p2 << "," << r2 << std::endl;
//
//  if( (dz>epsilon) || (dy>epsilon) || (dx>epsilon))
//  {
//    std::cout << std::scientific;
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "Test failed" << std::endl;
//    std::cout << dz << std::endl;
//    std::cout << dy << std::endl;
//    std::cout << dx << std::endl;
//    std::cout << std::setprecision(5) << std::fixed;
//    std::cout << "Original state: " << y << "," << p << "," << r << std::endl;
//    std::cout << "Convertd state: " << y2 << "," << p2 << "," << r2 << std::endl;
//    std::cout << std::string(80, '-') << std::endl;
//    exit(0);
//  }
//}
//void testRotationConversion(){
//  Srand(0);
//  for(int i = 0; i < 100; i++){
//
//    double roll = Rand(-M_PI,M_PI);
//
//    double pitch = Rand(-M_PI/2,M_PI/2);
//
//    double yaw = Rand(-M_PI,M_PI);
//
//    Math3D::EulerAngleRotation Reuler(roll, pitch, yaw);
//    Math3D::Matrix3 Rin;
//    Reuler.getMatrixXYZ(Rin);
//
//    double qx,qy,qz,qw;
//    Math3D::QuaternionRotation qr;
//    qr.setMatrix(Rin);
//    qr.get(qw,qx,qy,qz);
//
//    Math3D::QuaternionRotation qout(qw, qx, qy, qz);
//    Math3D::Matrix3 qR;
//    qout.getMatrix(qR);
//
//    Math3D::EulerAngleRotation ReulerOut;
//    ReulerOut.setMatrixXYZ(qR);
//
//    Math3D::Matrix3 Rout;
//    ReulerOut.getMatrixXYZ(Rout);
//
//    double yaw2 = ReulerOut[2];
//    double pitch2 = ReulerOut[1];
//    double roll2 = ReulerOut[0];
//
//    if(pitch2<-M_PI/2) pitch2+=M_PI;
//    if(pitch2>M_PI/2) pitch2-=M_PI;
//    if(roll2<-M_PI) roll2+=2*M_PI;
//    if(roll2>M_PI) roll2-=2*M_PI;
//    if(yaw2<-M_PI) yaw2+=2*M_PI;
//    if(yaw2>M_PI) yaw2-=2*M_PI;
//
//    double epsilon = 1e-10;
//    if(!Rin.isEqual(Rout, epsilon)){
//      std::cout << std::string(80, '-') << std::endl;
//      std::cout << "Matrices differ" << std::endl;
//      std::cout << Rin << std::endl;
//      std::cout << std::string(80, '-') << std::endl;
//      std::cout << Rout << std::endl;
//      std::cout << std::string(80, '-') << std::endl;
//      checkYawPitchRoll(yaw,pitch,roll,yaw2,pitch2,roll2);
//      exit(0);
//    }
//
//    checkYawPitchRoll(yaw,pitch,roll,yaw2,pitch2,roll2);
//  
//  }
//}
//void MotionPlannerOMPL::test_conversion(Config &q, ob::StateSpacePtr &stateSpace)
//{
//  ob::ScopedState<> qm = ConfigToOMPLState(q, stateSpace);
//  Config qq = OMPLStateToConfig(qm, stateSpace);
//
//  double epsilon = 1e-10;
//  if((q-qq).norm()>epsilon){
//    std::cout << "Klampt->OMPL->Klampt Test failed" << std::endl;
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "Klampt Input  state: " << q << std::endl;
//    std::cout << "Klampt Output state: " << qq << std::endl;
//    std::cout << std::string(80, '-') << std::endl;
//    exit(0);
//  }
//  ob::ScopedState<> qmout = ConfigToOMPLState(qq, stateSpace);
//  if(qm != qmout){
//    std::cout << "OMPL->Klampt->OMPL Test failed" << std::endl;
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "OMPL Input  state: " << qm << std::endl;
//    std::cout << "OMPL Output state: " << qmout << std::endl;
//    std::cout << std::string(80, '-') << std::endl;
//    exit(0);
//  }
//
//}

void SentinelPropagator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
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
  uSE3(0) = ucontrol[0];
  uSE3(1) = ucontrol[1];
  uSE3(2) = ucontrol[2];
  uSE3(3) = ucontrol[3];
  uSE3(4) = ucontrol[4];
  uSE3(5) = ucontrol[5];

  uint N = s->getDimension() - 6;

  //###########################################################################
  // Forward Simulate
  //###########################################################################

  uint Nduration = N+6;
  Real dt = ucontrol[Nduration];
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }

  LieGroupIntegrator integrator;

  Matrix4 x0_SE3 = integrator.StateToSE3(x0);
  Matrix4 dp0 = integrator.SE3Derivative(uSE3);
  Matrix4 x1_SE3 = integrator.Integrate(x0_SE3,dp0,dt);

  State x1(x0);
  integrator.SE3ToState(x1, x1_SE3);

  Config qend = x1;

  for(int i = 0; i < N; i++){
    qend[i+6] = x0[i+6] + dt*ucontrol[i+6];
  }

  //###########################################################################
  // Config to OMPL
  //###########################################################################

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
      Config cur = OMPLStateToConfig(state, cspace->getPtr());
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
  SingleRobotCSpace geometric_cspace = SingleRobotCSpace(*_world,_irobot,&worldsettings);
  if(!IsFeasible( robot, geometric_cspace, _p_goal)) return false;
  if(!IsFeasible( robot, geometric_cspace, _p_init)) return false;

  PrincipalFibreBundleAdaptor kcspace(&geometric_cspace);
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

  //###########################################################################
  // Kinodynamic planner
  //###########################################################################
  //KINODYNAMIC PLANNERS
  std::cout << robot->q.size() << std::endl;
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

  cbounds.setLow(NdimControl,0.01);//propagation step size
  cbounds.setHigh(NdimControl,0.10);

  control_cspace->setBounds(cbounds);

  oc::SimpleSetup ss(control_cspace);
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  //###########################################################################
  // choose planner
  //###########################################################################
  //const oc::SpaceInformationPtr si = ss.getSpaceInformation();
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::RRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::PDST>(si);
  ob::PlannerPtr ompl_planner = std::make_shared<oc::KPIECE1>(si);

  //###########################################################################
  // setup and projection
  //###########################################################################

  ss.setStateValidityChecker(std::make_shared<MotionPlannerOMPLValidityChecker>(si, &kcspace));

  auto cpropagate(std::make_shared<SentinelPropagator>(si, &kcspace));
  ss.setStatePropagator(cpropagate);

  double epsilon = 1.0;

  ss.setStartAndGoalStates(start, goal, epsilon);
  ss.setup();
  ss.setPlanner(ompl_planner);
  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

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

