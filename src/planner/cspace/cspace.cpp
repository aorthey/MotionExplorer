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
  world(world_), si(nullptr)
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
  if(si==nullptr){
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
const ob::StateValidityCheckerPtr CSpaceOMPL::StateValidityCheckerPtr()
{
  return StateValidityCheckerPtr(SpaceInformationPtr());
}

std::ostream& operator<< (std::ostream& out, const CSpaceOMPL& space) 
{
  out << std::string(80, '-') << std::endl;
  out << "[ConfigurationSpace]" << std::endl;
  out << std::string(80, '-') << std::endl;
  std::cout << "Robot                    : " << space.robot->name << std::endl;
  std::cout << "Dimensionality (OMPL)    : " << space.GetDimensionality() << std::endl;
  std::cout << "Dimensionality (Klampt)  : " << space.robot->q.size() << std::endl;
  //space.space->printSettings(std::cout);
  std::cout << "OMPL Representation      : " << std::endl;
  space.space->diagram(std::cout << "   ");

  out << std::string(80, '-') << std::endl;
  return out;
}
