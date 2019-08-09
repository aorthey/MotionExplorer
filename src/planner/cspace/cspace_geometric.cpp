#include "planner/cspace/cspace_geometric.h"

GeometricCSpaceOMPL::GeometricCSpaceOMPL(RobotWorld *world_, int robot_idx):
  CSpaceOMPL(world_, robot_idx)
{
}
bool GeometricCSpaceOMPL::isDynamic() const
{
  return false;
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
  ob::RealVectorStateSpace *cspaceRN = nullptr;

  if(Nompl>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl));
    this->space = SE3 + Rn;
    cspaceSE3 = this->space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);
    cspaceRN = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
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

void GeometricCSpaceOMPL::print() const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            :" << GetDimensionality() << std::endl;
  std::cout << " Configuration Space (klampt) : SE(3)" << (Nklampt>0?"xR^"+std::to_string(Nklampt):"") << "  [Klampt]"<< std::endl;
  std::cout << " Configuration Space (ompl)   : SE(3)" << (Nompl>0?"xR^"+std::to_string(Nompl):"") << "  [OMPL]" << std::endl;

  ob::SE3StateSpace *cspaceSE3 = nullptr;

  if(Nompl>0){
    cspaceSE3 = space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);
  }else{
    cspaceSE3 = space->as<ob::SE3StateSpace>();
  }
  si->printSettings();

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

  std::cout << std::string(80, '-') << std::endl;
}

void GeometricCSpaceOMPL::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
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
    qomplRnSpace = nullptr;
  }

  qomplSE3->setXYZ(q(0),q(1),q(2));
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5),qomplSO3);

  if(Nompl>0){
    double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
    for(uint i = 0; i < Nklampt; i++){
      int idx = klampt_to_ompl.at(i);
      if(idx<0) continue;
      else qomplRn[idx]=q(6+i);
    }
  }
}

Config GeometricCSpaceOMPL::OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState){
  const ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  Config q;
  q.resize(6+Nklampt);
  q.setZero();

  q(0) = qomplSE3->getX();
  q(1) = qomplSE3->getY();
  q(2) = qomplSE3->getZ();

  if(std::isnan((double)qomplSO3->x)){
    si->printSettings();
    std::cout << "SO3 is NaN" << std::endl;
    std::cout << qomplSO3->x << std::endl;
    std::cout << qomplSO3->y << std::endl;
    std::cout << qomplSO3->z << std::endl;
    std::cout << qomplSO3->w << std::endl;
    exit(0);
  }

  std::vector<double> rxyz = EulerXYZFromOMPLSO3StateSpace(qomplSO3);
  q(3) = rxyz.at(0);
  q(4) = rxyz.at(1);
  q(5) = rxyz.at(2);

  for(uint i = 0; i < Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    q(idx) = qomplRnState->values[i];
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
  OMPL_ERROR("GeometricCspace has no StatePropagatorPtr");
  exit(0);
}
