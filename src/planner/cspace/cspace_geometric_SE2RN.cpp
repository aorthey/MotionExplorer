#include "planner/cspace/cspace_geometric_SE2RN.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "ompl/base/spaces/SE2StateSpaceFullInterpolate.h"

GeometricCSpaceOMPLSE2RN::GeometricCSpaceOMPLSE2RN(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLSE2RN::initSpace()
{
  //###########################################################################
  // Create OMPL state space
  //   Create an SE(3) x R^n state space
  //###########################################################################
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "[MotionPlanner] only supports robots with a configuration space equal to SE(2) x R^n" << std::endl;
    throw "Invalid robot";
  }

  ob::StateSpacePtr SE2(std::make_shared<ob::SE2StateSpaceFullInterpolate>());
  ob::SE2StateSpaceFullInterpolate *cspaceSE2;
  ob::RealVectorStateSpace *cspaceRN = nullptr;

  if(Nompl>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl));
    this->space = SE2 + Rn;
    cspaceSE2 = this->space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpaceFullInterpolate>(0);
    cspaceRN = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }else{
    this->space = SE2;
    cspaceSE2 = this->space->as<ob::SE2StateSpaceFullInterpolate>();
  }

  //###########################################################################
  // Set bounds
  //###########################################################################

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> low;
  low.push_back(minimum.at(0));
  low.push_back(minimum.at(1));
  vector<double> high;
  high.push_back(maximum.at(0));
  high.push_back(maximum.at(1));

  ob::RealVectorBounds bounds(2);
  bounds.low = low;
  bounds.high = high;
  cspaceSE2->setBounds(bounds);
  bounds.check();

  if(cspaceRN!=nullptr)
  {
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

void GeometricCSpaceOMPLSE2RN::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SE2StateSpaceFullInterpolate::StateType *qomplSE2{nullptr};
  ob::RealVectorStateSpace::StateType *qomplRnSpace{nullptr};

  if(Nompl>0){
    qomplSE2 = qompl->as<ob::CompoundState>()->as<ob::SE2StateSpaceFullInterpolate::StateType>(0);
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }else{
    qomplSE2 = qompl->as<ob::SE2StateSpaceFullInterpolate::StateType>();
    qomplRnSpace = nullptr;
  }
  qomplSE2->setXY(q(0),q(1));
  qomplSE2->setYaw(q(3));

  if(Nompl>0){
    double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
    for(uint i = 0; i < Nklampt; i++){
      int idx = klampt_to_ompl.at(i);
      if(idx<0) continue;
      else qomplRn[idx]=q(6+i);
    }
  }

}

Config GeometricCSpaceOMPLSE2RN::OMPLStateToConfig(const ob::State *qompl){
  const ob::SE2StateSpaceFullInterpolate::StateType *qomplSE2{nullptr};
  const ob::RealVectorStateSpace::StateType *qomplRnSpace{nullptr};

  if(Nompl>0){
    qomplSE2 = qompl->as<ob::CompoundState>()->as<ob::SE2StateSpaceFullInterpolate::StateType>(0);
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }else{
    qomplSE2 = qompl->as<ob::SE2StateSpaceFullInterpolate::StateType>();
    qomplRnSpace = nullptr;
  }

  Config q;q.resize(robot->q.size());q.setZero();
  q(0)=qomplSE2->getX();
  q(1)=qomplSE2->getY();
  q(3)=qomplSE2->getYaw();

  for(uint i = 0; i < Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    q(idx) = qomplRnSpace->values[i];
  }

  return q;
}

void GeometricCSpaceOMPLSE2RN::print(std::ostream& out) const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            : " << 3 << std::endl;

  ob::SE2StateSpaceFullInterpolate *cspace = this->space->as<ob::SE2StateSpaceFullInterpolate>();

  const ob::RealVectorBounds bounds = cspace->getBounds();
  std::vector<double> min = bounds.low;
  std::vector<double> max = bounds.high;
  std::cout << "RN bounds min     : ";
  for(uint i = 0; i < min.size(); i++){
    std::cout << " " << min.at(i);
  }
  std::cout << std::endl;

  std::cout << "RN bounds max     : ";
  for(uint i = 0; i < max.size(); i++){
    std::cout << " " << max.at(i);
  }
  std::cout << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}
