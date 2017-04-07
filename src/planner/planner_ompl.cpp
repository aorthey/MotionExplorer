#include "planner_ompl.h"
#include "cspace_sentinel.h"

ob::PlannerTerminationCondition epsilonSolnPlannerTerminationCondition(ob::ProblemDefinitionPtr pdef, double epsilon)
{
    return ob::PlannerTerminationCondition([pdef, epsilon]
                                       {
                                          double d = pdef->getSolutionDifference();
                                          if(d<0) return false;
                                          return (d < epsilon);
                                       });
}

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

  //std::cout << startRn->getDimension() << std::endl;

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
ob::State* ConfigToOMPLStatePtr(const Config &q, const ob::StateSpacePtr &s){
  ob::ScopedState<> qompl = ConfigToOMPLState(q, s);

  ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();
  ob::RealVectorStateSpace::StateType *qomplRn = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  //double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;

  ob::State* out = s->allocState();
  ob::SE3StateSpace::StateType *outSE3 = out->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *outSO3 = &outSE3->rotation();
  ob::RealVectorStateSpace::StateType *outRn = out->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

  outSE3 = qomplSE3;
  outSO3 = qomplSO3;
  outRn = qomplRn;

  return out;
}
ob::ScopedState<> ConfigToOMPLState(const Config &q, const ob::StateSpacePtr &s){
  ob::ScopedState<> qompl(s);

  ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();
  ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;

  qomplSE3->setXYZ(q[0],q[1],q[2]);
  qomplSO3->setIdentity();

  //q SE3: X Y Z yaw pitch roll
  //double yaw = q[3];
  //double pitch = q[4];
  //double roll = q[5];

  Math3D::EulerAngleRotation Reuler(q(5),q(4),q(3));
  Matrix3 R;
  Reuler.getMatrixXYZ(R);

  QuaternionRotation qr;
  qr.setMatrix(R);

  double qx,qy,qz,qw;
  qr.get(qw,qx,qy,qz);

  qomplSO3->x = qx;
  qomplSO3->y = qy;
  qomplSO3->z = qz;
  qomplSO3->w = qw;

  //Math3D::Matrix3 qrM;
  //qr.getMatrix(qrM);

  for(int i = 0; i < q.size()-6; i++){
    qomplRn[i]=q(6+i);
  }
  return qompl;
}
Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::StateSpacePtr &s){
  const ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  //std::vector<double> reals;
  //s->copyToReals(reals, qomplRnState);
  uint N =  s->getDimension() - 6;
  Config q;
  q.resize(6+N);

  for(int i = 0; i < N; i++){
    q(i+6) = qomplRnState->values[i];
  }

  q(0) = qomplSE3->getX();
  q(1) = qomplSE3->getY();
  q(2) = qomplSE3->getZ();

  double qx = qomplSO3->x;
  double qy = qomplSO3->y;
  double qz = qomplSO3->z;
  double qw = qomplSO3->w;

  Math3D::QuaternionRotation qr(qw, qx, qy, qz);
  Math3D::Matrix3 qrM;
  qr.getMatrix(qrM);
  Math3D::EulerAngleRotation R;
  R.setMatrixXYZ(qrM);

  q(3) = R[2];
  q(4) = R[1];
  q(5) = R[0];

  if(q(3)<-M_PI) q(3)+=2*M_PI;
  if(q(3)>M_PI) q(3)-=2*M_PI;

  if(q(4)<-M_PI/2) q(4)+=M_PI;
  if(q(4)>M_PI/2) q(4)-=M_PI;

  if(q(5)<-M_PI) q(5)+=2*M_PI;
  if(q(5)>M_PI) q(5)-=2*M_PI;

  return q;
}
Config OMPLStateToConfig(const ob::State *qompl, const ob::StateSpacePtr &s){
  const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  return OMPLStateToConfig(qomplSE3, qomplRnState, s);

}

Config OMPLStateToConfig(const ob::ScopedState<> &qompl, const ob::StateSpacePtr &s){

  const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  return OMPLStateToConfig(qomplSE3, qomplRnState, s);
}

void MotionPlannerOMPL::SerializeTree(ob::PlannerData &pd)
{
  std::cout << "serializing tree with " << pd.numVertices() << " nodes" << std::endl;
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

    //std::cout << "Node " << i << " has " << Nedges << " edges" << std::endl;
    //for(int j = 0; j < edgeList.size(); j++){
    //  ob::PlannerDataVertex w = pd.getVertex(j);
    //  const ob::State* sw = w.getState();
    //  const ob::SE3StateSpace::StateType *swSE3 = sw->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    //  double xw = swSE3->getX();
    //  double yw = swSE3->getY();
    //  double zw = swSE3->getZ();

    //  Vector3 dvw( xw-x, yw-y, zw-z);
    //  snode.directions.push_back(dvw);
    //}



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
  return _space->IsFeasible(q);
}

MotionPlannerOMPL::MotionPlannerOMPL(RobotWorld *world, WorldSimulation *sim):
  MotionPlanner(world,sim)
{
}
void MotionPlannerOMPL::testSE3(KinodynamicCSpaceSentinelAdaptor &cspace)
{
  std::cout << "Testing Klampt->SE3->Klampt conversion" << std::endl;

  Srand(0);
  for(int i = 0; i < 100; i++){
    uint N = Rand(0,0);
    ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));
    ob::StateSpacePtr stateSpace = SE3 + Rn;

    Config q;
    q.resize(6+N);
    q.setZero();
    q[0]=Rand(-3,3);
    q[1]=Rand(-3,3);
    q[2]=Rand(-3,3);
    q[3] = Rand(-M_PI,M_PI);
    q[4] = Rand(-M_PI/2,M_PI/2);
    q[5] = Rand(-M_PI,M_PI);
    for(int j = 0; j < N; j++){
      q[j+6] = Rand(0,3);
    }
    Matrix4 x = cspace.StateToSE3(q);
    Config qq(q);
    qq.setZero();
    cspace.SE3ToState(qq,x);

    double epsilon = 1e-10;
    if((q-qq).norm()>epsilon){
      std::cout << "Klampt->SE3->Klampt Test "<<i<<" failed" << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Klampt Input  state: " << q << std::endl;
      std::cout << "Klampt Output state: " << qq << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      exit(0);
    }
  }

}

void MotionPlannerOMPL::test()
{
  std::cout << "Testing Klampt->OMPL->Klampt conversion" << std::endl;

  Srand(0);
  for(int i = 0; i < 100; i++){
    uint N = Rand(0,0);
    ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));
    ob::StateSpacePtr stateSpace = SE3 + Rn;

    Config q;
    q.resize(6+N);
    q.setZero();
    q[0]=Rand(-3,3);
    q[1]=Rand(-3,3);
    q[2]=Rand(-3,3);
    q[3] = Rand(-M_PI,M_PI);
    q[4] = Rand(-M_PI/2,M_PI/2);
    q[5] = Rand(-M_PI,M_PI);
    for(int j = 0; j < N; j++){
      q[j+6] = Rand(0,3);
    }
    //std::cout << "testing " << i << " : " << std::endl;
    test_conversion(q, stateSpace);
  }


}
void checkYawPitchRoll(double y, double p, double r, double y2, double p2, double r2)
{
  double epsilon = 1e-10;
  double dz = fabs(y-y2);
  double dy = fabs(p-p2);
  double dx = fabs(r-r2);
  //std::cout << "Original state: " << y << "," << p << "," << r << std::endl;
  //std::cout << "Convertd state: " << y2 << "," << p2 << "," << r2 << std::endl;

  if( (dz>epsilon) || (dy>epsilon) || (dx>epsilon))
  {
    std::cout << std::scientific;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Test failed" << std::endl;
    std::cout << dz << std::endl;
    std::cout << dy << std::endl;
    std::cout << dx << std::endl;
    std::cout << std::setprecision(5) << std::fixed;
    std::cout << "Original state: " << y << "," << p << "," << r << std::endl;
    std::cout << "Convertd state: " << y2 << "," << p2 << "," << r2 << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }
}
void testRotationConversion(){
  Srand(0);
  for(int i = 0; i < 100; i++){

    double roll = Rand(-M_PI,M_PI);

    double pitch = Rand(-M_PI/2,M_PI/2);

    double yaw = Rand(-M_PI,M_PI);

    Math3D::EulerAngleRotation Reuler(roll, pitch, yaw);
    Math3D::Matrix3 Rin;
    Reuler.getMatrixXYZ(Rin);

    double qx,qy,qz,qw;
    Math3D::QuaternionRotation qr;
    qr.setMatrix(Rin);
    qr.get(qw,qx,qy,qz);

    Math3D::QuaternionRotation qout(qw, qx, qy, qz);
    Math3D::Matrix3 qR;
    qout.getMatrix(qR);

    Math3D::EulerAngleRotation ReulerOut;
    ReulerOut.setMatrixXYZ(qR);

    Math3D::Matrix3 Rout;
    ReulerOut.getMatrixXYZ(Rout);

    double yaw2 = ReulerOut[2];
    double pitch2 = ReulerOut[1];
    double roll2 = ReulerOut[0];

    if(pitch2<-M_PI/2) pitch2+=M_PI;
    if(pitch2>M_PI/2) pitch2-=M_PI;
    if(roll2<-M_PI) roll2+=2*M_PI;
    if(roll2>M_PI) roll2-=2*M_PI;
    if(yaw2<-M_PI) yaw2+=2*M_PI;
    if(yaw2>M_PI) yaw2-=2*M_PI;

    double epsilon = 1e-10;
    if(!Rin.isEqual(Rout, epsilon)){
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Matrices differ" << std::endl;
      std::cout << Rin << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << Rout << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      checkYawPitchRoll(yaw,pitch,roll,yaw2,pitch2,roll2);
      exit(0);
    }

    checkYawPitchRoll(yaw,pitch,roll,yaw2,pitch2,roll2);
  
  }
}
void MotionPlannerOMPL::test_conversion(Config &q, ob::StateSpacePtr &stateSpace)
{
  ob::ScopedState<> qm = ConfigToOMPLState(q, stateSpace);
  Config qq = OMPLStateToConfig(qm, stateSpace);

  double epsilon = 1e-10;
  if((q-qq).norm()>epsilon){
    std::cout << "Klampt->OMPL->Klampt Test failed" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Klampt Input  state: " << q << std::endl;
    std::cout << "Klampt Output state: " << qq << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }
  ob::ScopedState<> qmout = ConfigToOMPLState(qq, stateSpace);
  if(qm != qmout){
    std::cout << "OMPL->Klampt->OMPL Test failed" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "OMPL Input  state: " << qm << std::endl;
    std::cout << "OMPL Output state: " << qmout << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }

}

void SentinelPropagator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
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

  //std::cout << "Propagate: " << std::endl;
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


bool MotionPlannerOMPL::solve(Config &p_init, Config &p_goal)
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
  cbounds.setLow(3,0.02);
  cbounds.setHigh(3,0.15);
  control_cspace->setBounds(cbounds);

  oc::SimpleSetup ss(control_cspace);
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  auto cpropagate(std::make_shared<SentinelPropagator>(si, &kcspace));

  //###########################################################################
  // choose planner
  //###########################################################################
  //const oc::SpaceInformationPtr si = ss.getSpaceInformation();
  ob::PlannerPtr ompl_planner = std::make_shared<oc::RRT>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::SST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::PDST>(si);
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::KPIECE1>(si);
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
  ss.setStartAndGoalStates(start, goal);
  ss.setup();
  ss.setPlanner(ompl_planner);

  //set default projection
  //ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //std::vector<double> cs(3); cs[0] = cs[1] = cs[2]= 0.1;
  //ss.getStateSpace()->getDefaultProjection()->setCellSizes(cs);
  //std::static_pointer_cast<oc::KPIECE1>(ompl_planner)->setProjectionEvaluator(ss.getStateSpace()->getDefaultProjection());

  //ob::ProjectionEvaluatorPtr projector = ss.getStateSpace()->getDefaultProjection();

  //if(!ss.getStateSpace()->hasDefaultProjection()){
  //  std::cout << "No default projection available" << std::endl;
  //}
  //###########################################################################
  // solve
  //
  // termination condition: 
  //    reached duration or found solution in epsilon-neighborhood
  //###########################################################################
  testRotationConversion();
  test();
  testSE3(kcspace);
  //KinodynamicCSpaceSentinelAdaptor kcspace(&geometric_cspace);
  //exit(0);
  double duration = 2.0;
  double epsilon = 0.1;

  ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(duration);
  //ob::PlannerTerminationCondition ptc_solution = ob::exactSolnPlannerTerminationCondition(ss.getProblemDefinition());
  ob::PlannerTerminationCondition ptc_solution = epsilonSolnPlannerTerminationCondition(ss.getProblemDefinition(), epsilon);
  ob::PlannerTerminationCondition ptc = ob::plannerOrTerminationCondition(ptc_time, ptc_solution);

  std::cout << "Starting Motion Planner" << std::endl;
  std::cout << "termination condition: reaching "<<duration << " seconds or being in epsilon="<< epsilon<<" neighborhood of solution" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  ob::PlannerStatus status = ss.solve(ptc);
  std::cout << "Status:" << status << std::endl;
  bool solved = ss.haveSolutionPath();

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
  // solve
  //###########################################################################

  if (solved)
  {
    std::cout << "Found solution:" << std::endl;
    oc::PathControl path_control = ss.getSolutionPath();
    og::PathGeometric path = path_control.asGeometric();
    std::cout << path.length() << std::endl;
    std::cout << path.getStateCount() << std::endl;

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = OMPLStateToConfig(state, cspace.getPtr());
      _keyframes.push_back(cur);
    }
    ob::State *obgoal = path.getState(path.getStateCount()-1);
    Config plannergoal = OMPLStateToConfig(obgoal, cspace.getPtr());

    double dgoal = kcspace.Distance(plannergoal, p_goal);
    std::cout << "Distance to goal: " << dgoal << std::endl;

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

