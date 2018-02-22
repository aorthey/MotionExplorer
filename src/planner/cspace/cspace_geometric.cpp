#include "planner/cspace/cspace_geometric.h"
#include "planner/validitychecker/validity_checker_ompl.h"

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
  //ob::RealVectorStateSpace *cspaceRn = nullptr;

  if(Nompl>0){
    cspaceSE3 = space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);
    //cspaceRn = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
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
    qomplRnSpace = nullptr;
  }

  qomplSE3->setXYZ(q[0],q[1],q[2]);
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5),qomplSO3);

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

  Config q;
  q.resize(6+Nklampt);
  q.setZero();

  q(0) = qomplSE3->getX();
  q(1) = qomplSE3->getY();
  q(2) = qomplSE3->getZ();

  if(std::isnan((double)qomplSO3->x)){
    si->printSettings();
    std::cout << qomplSO3 << std::endl;
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
  return std::make_shared<PrincipalFibreBundleIntegrator>(si, this);
}
const ob::StateValidityCheckerPtr GeometricCSpaceOMPL::StateValidityCheckerPtr(ob::SpaceInformationPtr si)
{
  return std::make_shared<OMPLValidityChecker>(si, this, kspace);
}
