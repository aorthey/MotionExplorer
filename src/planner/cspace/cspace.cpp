#include "gui/colors.h"
#include "gui/drawMotionPlanner.h"
#include "planner/cspace/cspace.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "planner/cspace/validitychecker/validity_checker_ompl_relaxation.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

CSpaceOMPL::CSpaceOMPL(RobotWorld *world_, int robot_idx_):
  si(nullptr), world(world_), robot_idx(robot_idx_)
{
  if(robot_idx < 0) return;

  robot = world->robots[robot_idx];
  worldsettings.InitializeDefault(*world);
  klampt_cspace = new SingleRobotCSpace(*world, robot_idx, &worldsettings);

  Nklampt =  robot->q.size() - 6;
  //check if the robot is SE(3) or if we need to add real vector space for joints
  if(Nklampt<=0){
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

    //prune dimensions which are smaller than epsilon (for ompl)
    double epsilonSpacing=1e-10;
    Nompl = 0;
    for(uint i = 6; i < 6+Nklampt; i++){

      if(abs(minimum.at(i)-maximum.at(i))>epsilonSpacing){
        klampt_to_ompl.push_back(Nompl);
        ompl_to_klampt.push_back(i);
        Nompl++;
      }else{
        klampt_to_ompl.push_back(-1);
      }
    }
  }
}
Config CSpaceOMPL::ControlToConfig(const double* control)
{
  Config q; q.setZero();
  OMPL_ERROR("NYI");
  throw "NYI";
  return q;
}

bool CSpaceOMPL::UpdateRobotConfig(Config &q)
{
  robot->UpdateConfig(q);
  robot->UpdateGeometry();
  return true;
}
bool CSpaceOMPL::SatisfiesBounds(const ob::State *state)
{
  return SpaceInformationPtr()->satisfiesBounds(state);
}

std::string CSpaceOMPL::GetName()
{
  if(robot) return robot->name;
  else return "unknown";
}
bool CSpaceOMPL::isMultiAgent() const
{
  return false;
}

ob::ScopedState<> CSpaceOMPL::ConfigVelocityToOMPLState(const Config &q, const Config &dq)
{
  ob::ScopedState<> qompl(space);
  ConfigVelocityToOMPLState(q, dq, qompl.get());
  return qompl;
}

void CSpaceOMPL::ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl)
{
  return ConfigToOMPLState(q, qompl);
}


Config CSpaceOMPL::OMPLStateToVelocity(const ob::State *)
{
  Config dq; dq.resize(GetKlamptDimensionality()); dq.setZero();
  return dq;
}

Config CSpaceOMPL::OMPLStateToConfig(const ob::ScopedState<> &qompl){
  const ob::State* s = qompl.get();
  return OMPLStateToConfig(s);
}

ob::ScopedState<> CSpaceOMPL::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  ConfigToOMPLState(q, qompl.get());
  return qompl;
}

void CSpaceOMPL::Init()
{
  this->initSpace();
}

bool CSpaceOMPL::isTimeDependent()
{
  return false;
}

double CSpaceOMPL::GetTime(const ob::State *qompl)
{
  return 0;
}

ob::SpaceInformationPtr CSpaceOMPL::SpaceInformationPtr()
{
  if(!si){
    si = std::make_shared<ob::SpaceInformation>(SpacePtr());
    validity_checker = StateValidityCheckerPtr(si);
    si->setStateValidityChecker(validity_checker);
  }
  return si;
}

const ob::StateSpacePtr CSpaceOMPL::SpacePtr()
{
  return space;
}

ob::StateSpacePtr CSpaceOMPL::SpacePtrNonConst()
{
  return space;
}

uint CSpaceOMPL::GetDimensionality() const
{
  return space->getDimension();
}

int CSpaceOMPL::GetRobotIndex() const
{
  return robot_idx;
}

uint CSpaceOMPL::GetKlamptDimensionality() const
{
  return Nklampt+6;
}

const oc::ControlSpacePtr CSpaceOMPL::ControlSpacePtr()
{
  return control_space;
}

uint CSpaceOMPL::GetControlDimensionality() const
{
  return control_space->getDimension();
}

void CSpaceOMPL::SetCSpaceInput(const CSpaceInput &input_)
{
  input = input_;
  fixedBase = input.fixedBase;
}

Robot* CSpaceOMPL::GetRobotPtr()
{
  return robot;
}

RobotWorld* CSpaceOMPL::GetWorldPtr()
{
  return world;
}

CSpace* CSpaceOMPL::GetCSpaceKlamptPtr()
{
  return klampt_cspace;
}

const ob::StateValidityCheckerPtr CSpaceOMPL::StateValidityCheckerPtr()
{
  if(!validity_checker){
    si = SpaceInformationPtr();
    validity_checker = StateValidityCheckerPtr(si);
    si->setStateValidityChecker(validity_checker);
  }
  return validity_checker;
}

void CSpaceOMPL::setStateValidityCheckerConstraintRelaxation(ob::State *xCenter, double r)
{
  si = SpaceInformationPtr();
  validity_checker = std::make_shared<OMPLValidityCheckerRelaxation>(si, this, xCenter, r);
  si->setStateValidityChecker(validity_checker);
}

const ob::StateValidityCheckerPtr CSpaceOMPL::StateValidityCheckerPtr(ob::SpaceInformationPtr si)
{
  if(enableSufficiency)
  {
    validity_checker = std::make_shared<OMPLValidityCheckerNecessarySufficient>(si, this, klampt_cspace_outer);
  }else{
    validity_checker = std::make_shared<OMPLValidityChecker>(si, this);
  }
  return validity_checker;
}

void CSpaceOMPL::SetSufficient(const uint robot_idx_outer_)
{
  robot_idx_outer = robot_idx_outer_;
  if(robot_idx_outer != robot_idx){
    klampt_cspace_outer = new SingleRobotCSpace(*world, robot_idx_outer, &worldsettings);
    enableSufficiency = true;
  }
}

bool CSpaceOMPL::isFixedBase()
{
  return fixedBase;
}
bool CSpaceOMPL::isFreeFloating()
{
  return !fixedBase;
}

bool CSpaceOMPL::IsPlanar()
{
  if(GetDimensionality()<=2) return true;

  ob::StateSpacePtr space_first_subspace = GetFirstSubspace();
  bool is_planar = (space_first_subspace->getType() == ob::STATE_SPACE_SE2);
  return is_planar;
}

ob::StateSpacePtr CSpaceOMPL::GetFirstSubspace()
{
  ob::StateSpacePtr space_first_subspace;
  //extract first component subspace
  if(!space->isCompound()){
    space_first_subspace = space;
  }else{
    if( (space->getType() == ob::STATE_SPACE_SE2) ||
        (space->getType() == ob::STATE_SPACE_SE3)){
      space_first_subspace = space;
    }else{
      ob::CompoundStateSpace *M1_compound = space->as<ob::CompoundStateSpace>();
      const std::vector<ob::StateSpacePtr> decomposed = M1_compound->getSubspaces();
      space_first_subspace = decomposed.front();
    }
  }
  return space_first_subspace;
}

Vector3 CSpaceOMPL::getXYZ(const ob::State *s, int ridx)
{
  return getXYZ(s);
}

void CSpaceOMPL::DrawGL(GUIState&)
{
  return;
}

void CSpaceOMPL::drawConfig(const Config &q, GLColor color, double scale)
{
    GLDraw::drawRobotAtConfig(robot, q, color);
}

void CSpaceOMPL::drawConfig(const Config &q, const Config &dq, GLColor color)
{
    GLDraw::drawRobotAtConfig(robot, q, dq, color);
}

std::vector<double> CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace( const ob::SO3StateSpace::StateType *q )
{
  double qx = q->x;
  double qy = q->y;
  double qz = q->z;
  double qw = q->w;

  double d = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if(abs(d-1.0) > 1e-10){
    //scale them down
    //if(d<0){
    //  d*=-1;
    //}
    qx /= d;
    qy /= d;
    qz /= d;
    qw /= d;
  }

  Math3D::QuaternionRotation qr(qw, qx, qy, qz);
  Math3D::Matrix3 qrM;
  qr.getMatrix(qrM);
  Math3D::EulerAngleRotation R;
  bool Rvalid = R.setMatrixXYZ(qrM);
  if(!Rvalid){

    std::cout << "quaternions: " << qr << std::endl;
    std::cout << qrM << std::endl;
    Real b=Asin(qrM(0,2));  //m(0,2)=sb
    Real cb = Cos(b);
    Real ca = qrM(2,2)/cb;   //m(2,2)=ca*cb
    std::cout << ca << std::endl;

    std::cout << "QuaternionRotation to EulerAngle not valid" << std::endl;
    throw "Invalid quaternion element.";
  }

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

void CSpaceOMPL::OMPLSO3StateSpaceFromEulerXYZ( double x, double y, double z, ob::SO3StateSpace::StateType *q )
{
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

void CSpaceOMPL::print(std::ostream& out) const
{
  out << std::string(80, '-') << std::endl;
  out << "[ConfigurationSpace]" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << "Robot                    : " << robot->name << std::endl;
  out << "Dimensionality (OMPL)    : " << GetDimensionality() << std::endl;
  out << "Dimensionality (Klampt)  : " << robot->q.size() << std::endl;
  out << "OMPL Representation      : " << std::endl;
  space->diagram(out << "   ");
  out << std::string(80, '-') << std::endl;
}

std::ostream& operator<< (std::ostream& out, const CSpaceOMPL& space) 
{
  space.print(out);
  return out;
}
