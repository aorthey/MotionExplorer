#include "planner/cspace/cspace.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>

std::vector<double> CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace( const ob::SO3StateSpace::StateType *q ){
  double qx = q->x;
  double qy = q->y;
  double qz = q->z;
  double qw = q->w;

  Math3D::QuaternionRotation qr(qw, qx, qy, qz);
  Math3D::Matrix3 qrM;
  qr.getMatrix(qrM);
  Math3D::EulerAngleRotation R;
  R.setMatrixXYZ(qrM);

  double rx = R[2];
  double ry = R[1];
  double rz = R[0];

  if(rx<-M_PI) rx+=2*M_PI;
  if(rx>M_PI) rx-=2*M_PI;

  if(ry<-M_PI/2) ry+=M_PI;
  if(ry>M_PI/2) ry-=M_PI;

  if(rz<-M_PI) rz+=2*M_PI;
  if(rz>M_PI) rz-=2*M_PI;

  std::vector<double> out;
  out.push_back(rx);
  out.push_back(ry);
  out.push_back(rz);
  return out;
}
void CSpaceOMPL::OMPLSO3StateSpaceFromEulerXYZ( double x, double y, double z, ob::SO3StateSpace::StateType *q ){
  q->setIdentity();

  //q SE3: X Y Z yaw pitch roll
  //double yaw = q[3];
  //double pitch = q[4];
  //double roll = q[5];

  //Math3D::EulerAngleRotation Reuler(q(5),q(4),q(3));
  //Matrix3 R;
  //Reuler.getMatrixXYZ(R);
  Math3D::EulerAngleRotation Reuler(z,y,x);
  Matrix3 R;
  Reuler.getMatrixXYZ(R);

  QuaternionRotation qr;
  qr.setMatrix(R);

  double qx,qy,qz,qw;
  qr.get(qw,qx,qy,qz);

  q->x = qx;
  q->y = qy;
  q->z = qz;
  q->w = qw;
}


CSpaceOMPL::CSpaceOMPL(RobotWorld *world_, int robot_idx):
  world(world_)
{
  robot = world->robots[robot_idx];
  worldsettings.InitializeDefault(*world);
  kspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);
}

CSpaceOMPL::CSpaceOMPL(Robot *robot_, CSpace *kspace_):
  robot(robot_), kspace(kspace_)
{
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "[MotionPlanner] only supports robots with a configuration space equal to SE(3) x R^n" << std::endl;
    exit(0);
  }
}
Config CSpaceOMPL::OMPLStateToConfig(const ob::ScopedState<> &qompl){
  const ob::State* s = qompl.get();
  return OMPLStateToConfig(s);
}

const ob::StateSpacePtr CSpaceOMPL::SpacePtr(){
  return space;
}
ob::SpaceInformationPtr CSpaceOMPL::SpaceInformationPtr(){
  if(!si){
    si = std::make_shared<ob::SpaceInformation>(SpacePtr());
    const ob::StateValidityCheckerPtr checker = StateValidityCheckerPtr(si);
    si->setStateValidityChecker(checker);
  }
  return si;
}
const oc::RealVectorControlSpacePtr CSpaceOMPL::ControlSpacePtr(){
  return control_space;
}
uint CSpaceOMPL::GetDimensionality() const{
  return space->getDimension();
}
uint CSpaceOMPL::GetControlDimensionality() const{
  return control_space->getDimension();
}
void CSpaceOMPL::SetCSpaceInput(const CSpaceInput &input_){
  input = input_;
}
Robot* CSpaceOMPL::GetRobotPtr(){
  return robot;
}
CSpace* CSpaceOMPL::GetCSpacePtr(){
  return kspace;
}

GeometricCSpaceOMPL::GeometricCSpaceOMPL(RobotWorld *world_, int robot_idx):
  CSpaceOMPL(world_, robot_idx)
{
  Init();
}
GeometricCSpaceOMPL::GeometricCSpaceOMPL(Robot *robot_, CSpace *kspace_):
  CSpaceOMPL(robot_, kspace_)
{
  Init();
}

void GeometricCSpaceOMPL::Init(){
  Nklampt =  robot->q.size() - 6;

  //check if the robot is SE(3) or if we need to add real vector space for joints
  if(Nklampt<=0){
    hasRealVectorSpace = false;
    Nompl = 0;
  }else{
    //sometimes the joint space are only fixed joints. In that case OMPL
    //complains that the real vector space is empty. We check here for that case
    //and remove the real vector space
    std::vector<double> minimum, maximum;
    minimum = robot->qMin;
    maximum = robot->qMax;
    assert(minimum.size() == 6+Nklampt);
    assert(maximum.size() == 6+Nklampt);

    //ompl does only accept dimensions with strictly positive measure, adding some epsilon space
    double epsilonSpacing=1e-10;
    hasRealVectorSpace = false;
    Nompl = 0;
    for(uint i = 6; i < 6+Nklampt; i++){

      if(abs(minimum.at(i)-maximum.at(i))>epsilonSpacing){
        hasRealVectorSpace = true;
        klampt_to_ompl.push_back(Nompl);
        ompl_to_klampt.push_back(i);
        Nompl++;
      }else{
        klampt_to_ompl.push_back(-1);
      }
    }
  }
}

void GeometricCSpaceOMPL::initSpace()
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

  ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
  ob::SE3StateSpace *cspaceSE3;
  ob::RealVectorStateSpace *cspaceRn;

  if(Nompl>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl));
    this->space = SE3 + Rn;
    cspaceSE3 = this->space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);
    cspaceRn = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }else{
    this->space = SE3;
    cspaceSE3 = this->space->as<ob::SE3StateSpace>();
  }

  //###########################################################################
  // Set bounds
  //###########################################################################
  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  assert(minimum.size() == 6+Nklampt);
  assert(maximum.size() == 6+Nklampt);

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
  boundsSE3.check();

  if(Nompl>0){
    vector<double> lowRn, highRn;

    for(uint i = 0; i < Nompl;i++){
      uint idx = ompl_to_klampt.at(i);
      double min = minimum.at(idx);
      double max = maximum.at(idx);
      lowRn.push_back(min);
      highRn.push_back(max);
    }
    ob::RealVectorBounds boundsRn(Nompl);

    ////ompl does only accept dimensions with strictly positive measure, adding some epsilon space
    //double epsilonSpacing=1e-10;
    //for(uint i = 0; i < N; i++){
    //  if(abs(lowRn.at(i)-highRn.at(i))<epsilonSpacing){
    //    highRn.at(i)=0;
    //  }
    //}

    boundsRn.low = lowRn;
    boundsRn.high = highRn;
    boundsRn.check();
    cspaceRn->setBounds(boundsRn);
  }

}
void GeometricCSpaceOMPL::initControlSpace()
{
  uint NdimControl = robot->q.size();

  this->control_space = std::make_shared<oc::RealVectorControlSpace>(space, NdimControl+1);
  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(-1);
  cbounds.setHigh(1);

  uint effectiveControlDim = 0;
  for(uint i = 0; i < NdimControl; i++){
    double qmin = robot->qMin(i);
    double qmax = robot->qMax(i);
    double d = sqrtf((qmin-qmax)*(qmin-qmax));
    if(d<1e-8){
      //remove zero-measure dimensions for control
      cbounds.setLow(i,0);
      cbounds.setHigh(i,0);
    }else{
      effectiveControlDim++;
      double dqmin = robot->velMin(i);
      double dqmax = robot->velMax(i);
      if(dqmin<-1) dqmin=-1;
      if(dqmax>1) dqmax=1;
      cbounds.setLow(i,dqmin);
      cbounds.setHigh(i,dqmax);
    }
  }

  cbounds.setLow(NdimControl,input.timestep_min);
  cbounds.setHigh(NdimControl,input.timestep_max);

  cbounds.check();
  control_space->setBounds(cbounds);
}

void GeometricCSpaceOMPL::print()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            :" << GetDimensionality() << std::endl;
  std::cout << " Configuration Space (original) : SE(3)" << (hasRealVectorSpace?"xR^"+std::to_string(Nklampt):"") << "  [Klampt]"<< std::endl;
  std::cout << " Configuration Space (effective): SE(3)" << (hasRealVectorSpace?"xR^"+std::to_string(Nompl):"") << "  [OMPL]" << std::endl;

  ob::SE3StateSpace *cspaceSE3 = NULL;
  ob::RealVectorStateSpace *cspaceRn = NULL;

  //uint N =  robot->q.size() - 6;
  if(Nompl>0){
    cspaceSE3 = space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);
    cspaceRn = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }else{
    cspaceSE3 = space->as<ob::SE3StateSpace>();
  }

//################################################################################
  const ob::RealVectorBounds bounds = cspaceSE3->getBounds();
  std::vector<double> se3min = bounds.low;
  std::vector<double> se3max = bounds.high;
  std::cout << "SE(3) bounds min     : ";
  for(uint i = 0; i < se3min.size(); i++){
    std::cout << " " << se3min.at(i);
  }
  std::cout << std::endl;

  std::cout << "SE(3) bounds max     : ";
  for(uint i = 0; i < se3max.size(); i++){
    std::cout << " " << se3max.at(i);
  }
  std::cout << std::endl;

  ob::RealVectorBounds cbounds = control_space->getBounds();
  std::vector<double> cbounds_low = cbounds.low;
  std::vector<double> cbounds_high = cbounds.high;

  std::cout << "Control bounds min   : ";
  for(uint i = 0; i < cbounds_low.size()-1; i++){
    std::cout << " " << cbounds_low.at(i);
  }
  std::cout << std::endl;
  std::cout << "Control bounds max   : ";
  for(uint i = 0; i < cbounds_high.size()-1; i++){
    std::cout << " " << cbounds_high.at(i);
  }
  std::cout << std::endl;
  std::cout << "Time step            : [" << cbounds_low.back()
    << "," << cbounds_high.back() << "]" << std::endl;

  std::cout << std::string(80, '-') << std::endl;
}

ob::ScopedState<> GeometricCSpaceOMPL::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);

  ob::SE3StateSpace::StateType *qomplSE3;
  ob::SO3StateSpace::StateType *qomplSO3;
  ob::RealVectorStateSpace::StateType *qomplRnSpace;

  if(Nompl>0){
    qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    qomplSO3 = &qomplSE3->rotation();
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }else{
    qomplSE3 = qompl->as<ob::SE3StateSpace::StateType>();
    qomplSO3 = &qomplSE3->rotation();
  }


  qomplSE3->setXYZ(q[0],q[1],q[2]);
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5),qomplSO3);

  //qomplSO3->setIdentity();

  ////q SE3: X Y Z yaw pitch roll
  ////double yaw = q[3];
  ////double pitch = q[4];
  ////double roll = q[5];

  //Math3D::EulerAngleRotation Reuler(q(5),q(4),q(3));
  //Matrix3 R;
  //Reuler.getMatrixXYZ(R);

  //QuaternionRotation qr;
  //qr.setMatrix(R);

  //double qx,qy,qz,qw;
  //qr.get(qw,qx,qy,qz);

  //qomplSO3->x = qx;
  //qomplSO3->y = qy;
  //qomplSO3->z = qz;
  //qomplSO3->w = qw;

  //Math3D::Matrix3 qrM;
  //qr.getMatrix(qrM);

  if(Nompl>0){
    double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
    for(uint i = 0; i < Nklampt; i++){
      int idx = klampt_to_ompl.at(i);
      if(idx<0) continue;
      else qomplRn[idx]=q(6+i);
    }
  }
  return qompl;
}
Config GeometricCSpaceOMPL::OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState){
  const ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  //std::vector<double> reals;
  //s->copyToReals(reals, qomplRnState);
  //uint N =  space->getDimension() - 6;

  Config q;
  q.resize(6+Nklampt);

  q(0) = qomplSE3->getX();
  q(1) = qomplSE3->getY();
  q(2) = qomplSE3->getZ();

  //double qx = qomplSO3->x;
  //double qy = qomplSO3->y;
  //double qz = qomplSO3->z;
  //double qw = qomplSO3->w;

  //Math3D::QuaternionRotation qr(qw, qx, qy, qz);
  //Math3D::Matrix3 qrM;
  //qr.getMatrix(qrM);
  //Math3D::EulerAngleRotation R;
  //R.setMatrixXYZ(qrM);

  //q(3) = R[2];
  //q(4) = R[1];
  //q(5) = R[0];

  //if(q(3)<-M_PI) q(3)+=2*M_PI;
  //if(q(3)>M_PI) q(3)-=2*M_PI;

  //if(q(4)<-M_PI/2) q(4)+=M_PI;
  //if(q(4)>M_PI/2) q(4)-=M_PI;

  //if(q(5)<-M_PI) q(5)+=2*M_PI;
  //if(q(5)>M_PI) q(5)-=2*M_PI;

  std::vector<double> rxyz = EulerXYZFromOMPLSO3StateSpace(qomplSO3);
  q(3) = rxyz.at(0);
  q(4) = rxyz.at(1);
  q(5) = rxyz.at(2);


  for(uint i = 0; i < Nklampt; i++){
    q(i+6) = 0;
  }

  if(Nompl>0){
    //set non-zero dimensions to ompl value
    for(uint i = 0; i < Nompl; i++){
      uint idx = ompl_to_klampt.at(i);
      q(idx) = qomplRnState->values[i];
    }
  }

  return q;
}
Config GeometricCSpaceOMPL::OMPLStateToConfig(const ob::State *qompl){
  if(Nompl>0){
    const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    return OMPLStateToConfig(qomplSE3, qomplRnState);
  }else{
    const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::SE3StateSpace::StateType>();
    return OMPLStateToConfig(qomplSE3, NULL);

  }

}

const oc::StatePropagatorPtr GeometricCSpaceOMPL::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  return std::make_shared<PrincipalFibreBundleIntegrator>(si, this);
}
const ob::StateValidityCheckerPtr GeometricCSpaceOMPL::StateValidityCheckerPtr(ob::SpaceInformationPtr si)
{
  return std::make_shared<OMPLValidityChecker>(si, this, kspace);
}
const ob::StateValidityCheckerPtr GeometricCSpaceOMPL::StateValidityCheckerPtr()
{
  return std::make_shared<OMPLValidityChecker>(SpaceInformationPtr(), this, kspace);
}
const ob::StateValidityCheckerPtr GeometricCSpaceOMPL::StateValidityCheckerPtr(oc::SpaceInformationPtr si)
{
  std::cout << "KinodynamicCSpaceOMPL only supports OMPL geometric (ob::). You called OMPL control (oc::)" << std::endl;
  exit(0);
}

//#############################################################################
//#############################################################################
//#############################################################################
KinodynamicCSpaceOMPL::KinodynamicCSpaceOMPL(Robot *robot_, CSpace *kspace_):
  CSpaceOMPL(robot_, kspace_)
{


}

void KinodynamicCSpaceOMPL::print()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Dimensionality Space      :" << GetDimensionality() << std::endl;
  std::cout << "Dimensionality Ctrl Space :" << GetControlDimensionality() << std::endl;
  ob::CompoundStateSpace *cspace = space->as<ob::CompoundStateSpace>();
  ob::SE3StateSpace *cspaceSE3 = cspace->as<ob::SE3StateSpace>(0);

  ob::RealVectorStateSpace *cspaceRn = NULL;
  ob::RealVectorStateSpace *cspaceTM = NULL;

  uint N =  robot->q.size() - 6;
  if(N>0){
    cspaceRn = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2);
  }else{
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }

//################################################################################
  const ob::RealVectorBounds bounds = cspaceSE3->getBounds();
  std::vector<double> se3min = bounds.low;
  std::vector<double> se3max = bounds.high;
  std::cout << "SE(3) bounds min     : ";
  for(uint i = 0; i < se3min.size(); i++){
    std::cout << " " << se3min.at(i);
  }
  std::cout << std::endl;

  std::cout << "SE(3) bounds max     : ";
  for(uint i = 0; i < se3max.size(); i++){
    std::cout << " " << se3max.at(i);
  }
  std::cout << std::endl;

//################################################################################
  const ob::RealVectorBounds boundstm = cspaceTM->getBounds();
  std::vector<double> min = boundstm.low;
  std::vector<double> max = boundstm.high;
  std::cout << "Vel bounds min       : ";
  for(uint i = 0; i < min.size(); i++){
    std::cout << " " << min.at(i);
  }
  std::cout << std::endl;
  std::cout << "Vel bounds max       : ";
  for(uint i = 0; i < max.size(); i++){
    std::cout << " " << max.at(i);
  }
  std::cout << std::endl;


  std::cout << std::string(80, '-') << std::endl;

}

void KinodynamicCSpaceOMPL::initSpace()
{
  //###########################################################################
  // Create OMPL state space
  //   Create an SE(3) x R^n space + a R^{6+N} space
  //###########################################################################
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "[MotionPlanner] only supports robots with a configuration space equal to SE(3) x R^n" << std::endl;
    exit(0);
  }

  double N = robot->q.size()-6;
  std::cout << "[MotionPlanner] Robot CSpace: M = SE(3) x R^" << N << std::endl;
  std::cout << "[MotionPlanner] Tangent Bundle: TM = M x R^{6+" << N << "}" << std::endl;

  ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());


  ob::StateSpacePtr TM(std::make_shared<ob::RealVectorStateSpace>(6+N));

  if(N>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));
    space = SE3 + Rn + TM;
  }else{
    space = SE3 + TM;
  }

  ob::SE3StateSpace *cspaceSE3 = space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);

  ob::RealVectorStateSpace *cspaceRn;
  ob::RealVectorStateSpace *cspaceTM;

  if(N>0){
    cspaceRn = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2);
  }else{
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }

  //###########################################################################
  // Set position bounds
  //###########################################################################
  std::vector<double> minimum, maximum;
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
  boundsSE3.check();
  cspaceSE3->setBounds(boundsSE3);

  if(N>0){
    vector<double> lowRn, highRn;
    for(uint i = 0; i < N; i++){
      lowRn.push_back(minimum.at(i+6));
      highRn.push_back(maximum.at(i+6));
    }
    ob::RealVectorBounds boundsRn(N);

    //ompl does only accept dimensions with strictly positive measure, adding some epsilon space
    double epsilonSpacing=1e-8;
    for(uint i = 0; i < N; i++){
      if(abs(lowRn.at(i)-highRn.at(i))<epsilonSpacing){
        highRn.at(i)+=epsilonSpacing;
      }
    }
    boundsRn.low = lowRn;
    boundsRn.high = highRn;
    boundsRn.check();
    cspaceRn->setBounds(boundsRn);
  }
  //###########################################################################
  // Set velocity bounds
  //###########################################################################
  std::vector<double> vMin, vMax;
  vMin = robot->velMin;
  vMax = robot->velMax;

  assert(vMin.size() == 6+N);
  assert(vMax.size() == 6+N);


  vector<double> lowTM, highTM;
  for(uint i = 0; i < 6+N; i++){
    lowTM.push_back(vMin.at(i));
    highTM.push_back(vMax.at(i));
  }

  ob::RealVectorBounds boundsTM(6+N);
  boundsTM.low = lowTM;
  boundsTM.high = highTM;
  boundsTM.check();
  //TODO
  boundsTM.setLow(-100);
  boundsTM.setHigh(100);
  cspaceTM->setBounds(boundsTM);

}
void KinodynamicCSpaceOMPL::initControlSpace(){
  uint NdimControl = robot->q.size();
  this->control_space = std::make_shared<oc::RealVectorControlSpace>(space, NdimControl+1);

  Vector torques = robot->torqueMax;
  //std::cout << "TORQUES robot:" << std::endl;
  //std::cout << robot->q.size() << std::endl;
  //std::cout << torques.size() << std::endl;
  //std::cout << torques << std::endl;

  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(-1);
  cbounds.setHigh(1);

  uint effectiveControlDim = 0;
  for(uint i = 0; i < NdimControl; i++){
    double qmin = robot->qMin(i);
    double qmax = robot->qMax(i);
    double d = sqrtf((qmin-qmax)*(qmin-qmax));
    if(d<1e-8){
      //remove zero-measure dimensions for control
      cbounds.setLow(i,0);
      cbounds.setHigh(i,0);
    }else{
      effectiveControlDim++;
      double tmin = -torques(i);
      double tmax = torques(i);
      //if(tmin<-1) tmin=-1;
      //if(tmax>1) tmax=1;
      cbounds.setLow(i,tmin);
      cbounds.setHigh(i,tmax);
    }
  }

  cbounds.setLow(NdimControl,input.timestep_min);//propagation step size
  cbounds.setHigh(NdimControl,input.timestep_max);

  //TODO: remove hardcoded se(3) vector fields
  for(uint i = 0; i < 6; i++){
    cbounds.setLow(i,0);
    cbounds.setHigh(i,0);
  }

  cbounds.setLow(0,1);
  cbounds.setHigh(0,1);
  //cbounds.setLow(1,-1);
  //cbounds.setHigh(1,1);

   //cbounds.setLow(3,-0.5);
   //cbounds.setHigh(3,-0.5);

  cbounds.setLow(4,-0.5);
  cbounds.setHigh(4,-0.5);

  //cbounds.setLow(5,-1);
  //cbounds.setHigh(5,1);

  cbounds.check();
  control_space->setBounds(cbounds);
  //std::cout << "torque bounds" << std::endl;
  //for(uint i = 0; i < NdimControl+1; i++){
  //  std::cout << i << " <" << cbounds.low.at(i) << ","<< cbounds.high.at(i) << ">" << std::endl;
  //}
}

ob::ScopedState<> KinodynamicCSpaceOMPL::ConfigToOMPLState(const Config &q){
  uint N=0;
  if(!(q.size() == int(space->getDimension()))){
    if(q.size() == int(0.5*space->getDimension())){
      N = q.size()-6;
    }else{
      exit(0);
    }
  }else{
    N = int(0.5*q.size())-6;
    assert(12+2*N == space->getDimension());
  }


  ob::ScopedState<> qompl(space);

  ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

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

  ob::RealVectorStateSpace::StateType *qomplRnSpace;
  ob::RealVectorStateSpace::StateType *qomplTMSpace;
  if(N>0){
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }
  double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
  //q.size = 6 + N + (N+6)

  if(!(q.size() == int(space->getDimension()))){
    if(q.size() == int(0.5*space->getDimension())){
      assert(12+2*N == space->getDimension());
      for(uint i = 0; i < N; i++){
        qomplRn[i]=q(6+i);
      }
      double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;
      for(uint i = 0; i < (N+6); i++){
        qomplTM[i]=0.0;
      }
    }else{
      exit(0);
    }
  }else{
    uint N = int(0.5*q.size())-6;
    assert(12+2*N == space->getDimension());
    for(uint i = 0; i < N; i++){
      qomplRn[i]=q(6+i);
    }
    double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;
    for(uint i = 0; i < (N+6); i++){
      qomplTM[i]=q(N+6+i);
    }
  }


  return qompl;
}


Config KinodynamicCSpaceOMPL::OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::RealVectorStateSpace::StateType *qomplTMState){
  const ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  //std::vector<double> reals;
  //s->copyToReals(reals, qomplRnState);
  uint N =  int(0.5*space->getDimension()) - 6;
  assert(12+2*N == space->getDimension());

  Config q;
  q.resize(12+2*N);

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

  for(uint i = 0; i < N; i++){
    q(i+6) = qomplRnState->values[i];
  }
  if(N>0){
    for(uint i = 0; i < (6+N); i++){
      q(i+6+N) = qomplTMState->values[i];
    }
  }

  return q;
}
Config KinodynamicCSpaceOMPL::OMPLStateToConfig(const ob::State *qompl){
  const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  const ob::RealVectorStateSpace::StateType *qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

  return OMPLStateToConfig(qomplSE3, qomplRnState, qomplTMState);

}

const oc::StatePropagatorPtr KinodynamicCSpaceOMPL::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  return std::make_shared<TangentBundleIntegrator>(si, robot, this);
}
const ob::StateValidityCheckerPtr KinodynamicCSpaceOMPL::StateValidityCheckerPtr(oc::SpaceInformationPtr si)
{
  return std::make_shared<TangentBundleOMPLValidityChecker>(si, kspace, this);
}
const ob::StateValidityCheckerPtr KinodynamicCSpaceOMPL::StateValidityCheckerPtr(ob::SpaceInformationPtr si)
{
  std::cout << "KinodynamicCSpaceOMPL only supports OMPL control (oc::). You called OMPL geometric (ob::)" << std::endl;
  exit(0);
}
//#############################################################################
//#############################################################################
GeometricCSpaceOMPLRotationalInvariance::GeometricCSpaceOMPLRotationalInvariance(Robot *robot_, CSpace *space_):
  GeometricCSpaceOMPL(robot_, space_)
{
}

void GeometricCSpaceOMPLRotationalInvariance::initSpace()
{
  //###########################################################################
  // Create OMPL state space
  //   Create an R^3 state
  //###########################################################################
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "[MotionPlanner] only supports robots with a configuration space equal to SE(3) x R^n" << std::endl;
    exit(0);
  }

  if(hasRealVectorSpace){
    std::cout << "Robot has real vector space, but should be a rotational invariant rigid body?" << std::endl;
    exit(0);
  }
  std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: R^3" << std::endl;

  ob::StateSpacePtr R3(std::make_shared<ob::RealVectorStateSpace>(3));
  this->space = R3;
  ob::RealVectorStateSpace *cspaceR3 = this->space->as<ob::RealVectorStateSpace>();

  //###########################################################################
  // Set bounds
  //###########################################################################
  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> lowR3;
  lowR3.push_back(minimum.at(0));
  lowR3.push_back(minimum.at(1));
  lowR3.push_back(minimum.at(2));
  vector<double> highR3;
  highR3.push_back(maximum.at(0));
  highR3.push_back(maximum.at(1));
  highR3.push_back(maximum.at(2));

  ob::RealVectorBounds boundsR3(3);
  boundsR3.low = lowR3;
  boundsR3.high = highR3;
  cspaceR3->setBounds(boundsR3);
  boundsR3.check();

}
void GeometricCSpaceOMPLRotationalInvariance::print()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Dimensionality Space      :" << GetDimensionality() << std::endl;

  ob::RealVectorStateSpace *cspaceRn = space->as<ob::RealVectorStateSpace>();

//################################################################################
  const ob::RealVectorBounds bounds = cspaceRn->getBounds();
  std::vector<double> se3min = bounds.low;
  std::vector<double> se3max = bounds.high;
  std::cout << "SE(3) bounds min     : ";
  for(uint i = 0; i < se3min.size(); i++){
    std::cout << " " << se3min.at(i);
  }
  std::cout << std::endl;

  std::cout << "SE(3) bounds max     : ";
  for(uint i = 0; i < se3max.size(); i++){
    std::cout << " " << se3max.at(i);
  }
  std::cout << std::endl;

  std::cout << std::string(80, '-') << std::endl;
}

ob::ScopedState<> GeometricCSpaceOMPLRotationalInvariance::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  for(uint i = 0; i < 3; i++){
    qompl->as<ob::RealVectorStateSpace::StateType>()->values[i] = q(i);
  }
  return qompl;
}

Config GeometricCSpaceOMPLRotationalInvariance::OMPLStateToConfig(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::RealVectorStateSpace::StateType>();
  Config q;q.resize(6);q.setZero();
  q(0)=qomplRnSpace->values[0];
  q(1)=qomplRnSpace->values[1];
  q(2)=qomplRnSpace->values[2];
  return q;

}

//#############################################################################
////#############################################################################
//GeometricCSpaceOMPLPathConstraintRollInvariance::GeometricCSpaceOMPLPathConstraintRollInvariance(Robot *robot_, CSpace *space_, std::vector<Config> path_):
//  GeometricCSpaceOMPL(robot_, space_)
//{
//  path_constraint = PathPiecewiseLinearEuclidean::from_keyframes(path_);
//  path_constraint->Normalize();
//}
//
//void GeometricCSpaceOMPLPathConstraintRollInvariance::initSpace()
//{
//  //###########################################################################
//  //   R^1 times S times S
//  //###########################################################################
//  //std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: [0,1] x S^1 x S^1" << std::endl;
//
//  ob::StateSpacePtr R = (std::make_shared<ob::RealVectorStateSpace>(1));
//  ob::StateSpacePtr S1a = (std::make_shared<ob::SO2StateSpace>());
//  ob::StateSpacePtr S1b = (std::make_shared<ob::SO2StateSpace>());
//  this->space = R + S1a + S1b;
//  ob::RealVectorStateSpace *cspaceR = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0);
//  //ob::SO2StateSpace *cspaceS1a = this->space->as<ob::CompoundStateSpace>()->as<ob::SO2StateSpace>(1);
//  //ob::SO2StateSpace *cspaceS1b = this->space->as<ob::CompoundStateSpace>()->as<ob::SO2StateSpace>(2);
//
//  ob::RealVectorBounds cbounds(1);
//  cbounds.setLow(0);
//  cbounds.setHigh(1+1e-10);
//  cspaceR->setBounds(cbounds);
//}
//
//void GeometricCSpaceOMPLPathConstraintRollInvariance::print()
//{
//}
//
//Config GeometricCSpaceOMPLPathConstraintRollInvariance::OMPLStateToConfig(const ob::State *qompl){
//
//  const ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
//  const ob::SO2StateSpace::StateType *qomplSO2SpaceA = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
//  const ob::SO2StateSpace::StateType *qomplSO2SpaceB = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2);
//
//  double t = qomplRnSpace->values[0];
//  double q1 = qomplSO2SpaceA->value;
//  double q2 = qomplSO2SpaceB->value;
//
//  Config qconstraint = path_constraint->Eval(t);
//
//  Config q;q.resize(6);q.setZero();
//  q(0)=qconstraint(0);
//  q(1)=qconstraint(1);
//  q(2)=qconstraint(2);
//  q(3)=q1;
//  q(4)=q2;
//  q(5)=0.0;
//
//  return q;
//}
//
//ob::ScopedState<> GeometricCSpaceOMPLPathConstraintRollInvariance::ConfigToOMPLState(const Config &q){
//  ob::ScopedState<> qompl(space);
//  ob::SO2StateSpace::StateType *qomplSO2SpaceA = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
//  ob::SO2StateSpace::StateType *qomplSO2SpaceB = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(2);
//  static_cast<ob::SO2StateSpace::StateType*>(qomplSO2SpaceA)->value = q[3];
//  static_cast<ob::SO2StateSpace::StateType*>(qomplSO2SpaceB)->value = q[4];
//
//  Config qc;qc.resize(3);qc.setZero();
//  qc(0)=q(0); qc(1)=q(1); qc(2)=q(2);
//
//  //qomplRn[0] = path_constraint->PosFromConfig(qc);
//  qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = path_constraint->PosFromConfig(qc);
//  return qompl;
//}
//#############################################################################
GeometricCSpaceOMPLPathConstraintSO3::GeometricCSpaceOMPLPathConstraintSO3(Robot *robot_, CSpace *space_, std::vector<Config> path_):
  GeometricCSpaceOMPL(robot_, space_)
{
  path_constraint = PathPiecewiseLinearEuclidean::from_keyframes(path_);
  path_constraint->Normalize();
}

void GeometricCSpaceOMPLPathConstraintSO3::initSpace()
{
  //###########################################################################
  // @brief $R^1 \times SO(3)$
  //###########################################################################
  std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: [0,1] x SO(3)" << std::endl;

  ob::StateSpacePtr R = (std::make_shared<ob::RealVectorStateSpace>(1));
  ob::StateSpacePtr SO3 = (std::make_shared<ob::SO3StateSpace>());
  this->space = R + SO3;
  ob::RealVectorStateSpace *cspaceR = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0);
  //ob::SO3StateSpace *cspaceSO3 = this->space->as<ob::CompoundStateSpace>()->as<ob::SO3StateSpace>(1);

  ob::RealVectorBounds cbounds(1);
  cbounds.setLow(0);
  cbounds.setHigh(1+1e-10);
  cspaceR->setBounds(cbounds);
}

void GeometricCSpaceOMPLPathConstraintSO3::print()
{
}

Config GeometricCSpaceOMPLPathConstraintSO3::OMPLStateToConfig(const ob::State *qompl){

  const ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  const ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(1);

  double t = qomplRnSpace->values[0];
  std::vector<double> rxyz = EulerXYZFromOMPLSO3StateSpace(qomplSO3);

  Config qconstraint = path_constraint->Eval(t);

  Config q;q.resize(6);q.setZero();
  q(0)=qconstraint(0);
  q(1)=qconstraint(1);
  q(2)=qconstraint(2);
  q(3)=rxyz.at(0);
  q(4)=rxyz.at(1);
  q(5)=rxyz.at(2);

  return q;
}

ob::ScopedState<> GeometricCSpaceOMPLPathConstraintSO3::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);

  Config qc;qc.resize(3);qc.setZero();
  qc(0)=q(0); qc(1)=q(1); qc(2)=q(2);

  qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = path_constraint->PosFromConfig(qc);

  ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(1);
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5),qomplSO3);

  return qompl;
}

//#############################################################################
//#############################################################################
GeometricCSpaceOMPLPointConstraintSO3::GeometricCSpaceOMPLPointConstraintSO3(Robot *robot_, CSpace *space_, Config q):
  GeometricCSpaceOMPL(robot_, space_), q_constraint(q)
{
}

void GeometricCSpaceOMPLPointConstraintSO3::initSpace()
{
  //###########################################################################
  //   {q} times SO(3)
  //###########################################################################
  std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: {q} x SO(3)" << std::endl;
  ob::StateSpacePtr SO3 = (std::make_shared<ob::SO3StateSpace>());
  this->space = SO3;
}

void GeometricCSpaceOMPLPointConstraintSO3::print()
{
}

Config GeometricCSpaceOMPLPointConstraintSO3::OMPLStateToConfig(const ob::State *qompl){

  const ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::SO3StateSpace::StateType>();

  std::vector<double> rxyz = EulerXYZFromOMPLSO3StateSpace(qomplSO3);

  Config q;q.resize(6);q.setZero();
  q(0)=q_constraint(0);
  q(1)=q_constraint(1);
  q(2)=q_constraint(2);
  q(3)=rxyz.at(0);
  q(4)=rxyz.at(1);
  q(5)=rxyz.at(2);

  return q;
}

ob::ScopedState<> GeometricCSpaceOMPLPointConstraintSO3::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::SO3StateSpace::StateType>();
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5),qomplSO3);
  return qompl;
}

std::ostream& operator<< (std::ostream& out, const CSpaceOMPL& space) 
{
  out << std::string(80, '-') << std::endl;
  out << "[ConfigurationSpace]" << std::endl;
  out << std::string(80, '-') << std::endl;
  std::cout << "Dimensionality (OMPL)    : " << space.GetDimensionality() << std::endl;
  //uint ii = space.input.robot_inner_idx;
  //uint io = space.input.robot_outer_idx;
  //std::cout << " robot inner : " << ii << " " << space.world->robots[ii]->name << std::endl;
  //std::cout << " robot outer : " << io << " " << space.world->robots[io]->name << std::endl;
  out << std::string(80, '-') << std::endl;
  return out;
}
