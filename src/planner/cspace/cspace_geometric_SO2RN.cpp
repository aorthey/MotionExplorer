#include "planner/cspace/cspace_geometric_SO2RN.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "common.h"

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/util/Exception.h>

GeometricCSpaceOMPLSO2RN::GeometricCSpaceOMPLSO2RN(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
  Nklampt =  robot->q.size() - 7;
  //check if the robot is SE(3) or if we need to add real vector space for joints
  klampt_to_ompl.clear();
  ompl_to_klampt.clear();
  if(Nklampt<=0){
    Nompl = 0;
  }else{
    std::vector<double> minimum, maximum;
    minimum = robot->qMin;
    maximum = robot->qMax;
    assert(minimum.size() == 7+Nklampt);
    assert(maximum.size() == 7+Nklampt);

    //prune dimensions which are smaller than epsilon (for ompl)
    double epsilonSpacing=1e-10;
    Nompl = 0;
    for(uint i = 7; i < 7+Nklampt; i++){

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

void GeometricCSpaceOMPLSO2RN::initSpace()
{
  //###########################################################################
  // Create OMPL state space
  //   Create an SO(2) x R^n state space
  //###########################################################################

  ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
  ob::RealVectorStateSpace *cspaceRN = nullptr;


  if(Nompl>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl));
    this->space = SO2 + Rn;
    cspaceRN = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }else{
    this->space = SO2;
  }

  //###########################################################################
  // Set bounds
  //###########################################################################
  if(cspaceRN!=nullptr)
  {
    std::vector<double> minimum, maximum;
    minimum = robot->qMin;
    maximum = robot->qMax;
    vector<double> lowRn, highRn;

    for(uint i = 0; i < Nompl;i++){
      uint idx = ompl_to_klampt.at(i);
      double min = minimum.at(idx);
      double max = maximum.at(idx);
      lowRn.push_back(min);
      highRn.push_back(max);
    }

    ob::RealVectorBounds boundsRn(Nompl);

    boundsRn.low = lowRn;
    boundsRn.high = highRn;
    boundsRn.check();
    cspaceRN->setBounds(boundsRn);
  }
}

void GeometricCSpaceOMPLSO2RN::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SO2StateSpace::StateType *qomplSO2{nullptr};
  ob::RealVectorStateSpace::StateType *qomplRnSpace{nullptr};

  if(Nompl>0){
    qomplSO2 = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }else{
    qomplSO2 = qompl->as<ob::SO2StateSpace::StateType>();
    qomplRnSpace = nullptr;
  }
  qomplSO2->value = q(6); //Note: this is fixedbase! So first joint value is q(6) in klampt

  if(Nompl>0){
    double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
    for(uint i = 0; i < Nklampt; i++){
      int idx = klampt_to_ompl.at(i);
      if(idx<0) continue;
      else qomplRn[idx]=q(7+i);
    }
  }

}

Config GeometricCSpaceOMPLSO2RN::OMPLStateToConfig(const ob::State *qompl)
{
  const ob::SO2StateSpace::StateType *qomplSO2{nullptr};
  const ob::RealVectorStateSpace::StateType *qomplRnSpace{nullptr};

  if(Nompl>0){
    qomplSO2 = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }else{
    qomplSO2 = qompl->as<ob::SO2StateSpace::StateType>();
    qomplRnSpace = nullptr;
  }

  Config q;q.resize(robot->q.size());q.setZero();
  q(6)=qomplSO2->value;

  for(uint i = 0; i < Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    q(idx) = qomplRnSpace->values[i];
  }

  return q;
}

void GeometricCSpaceOMPLSO2RN::print(std::ostream& out) const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            : " << 1+Nompl << std::endl;
}

Vector3 GeometricCSpaceOMPLSO2RN::getXYZ(const ob::State *s)
{
  if(!isFixedBase()){
    OMPL_ERROR("Undefined: Non-fixedbase robots need to start with spaces SE2 or SE3.");
    throw ompl::Exception("Undefined");
  }else{
    Config q = OMPLStateToConfig(s);
    robot->UpdateConfig(q);
    robot->UpdateGeometry();
    Vector3 qq;
    Vector3 zero; zero.setZero();
    int lastLink = robot->links.size()-1;

    //NOTE: the world position is zero exactly at the point where link is
    //attached using a joint to the whole linkage. Check where your last fixed
    //joint is positioned, before questioning the validity of this method
    robot->GetWorldPosition(zero, lastLink, qq);

    double x = qq[0];
    double y = qq[1];
    double z = qq[2];
    Vector3 v(x,y,z);
    return v;
  }
}
