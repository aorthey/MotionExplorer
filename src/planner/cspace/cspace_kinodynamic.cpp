#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/cspace/integrator/tangentbundle.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include "common.h"

KinodynamicCSpaceOMPL::KinodynamicCSpaceOMPL(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void KinodynamicCSpaceOMPL::print(std::ostream& out) const
{
  ob::CompoundStateSpace *cspace = space->as<ob::CompoundStateSpace>();
  ob::SE3StateSpace *cspaceSE3 = cspace->as<ob::SE3StateSpace>(0);

  ob::RealVectorStateSpace *cspaceTM = nullptr;

  if(Nompl>0){
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2);
  }else{
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }
//################################################################################
  std::cout << std::endl;
  std::cout << "Configuration Space M = SE(3) x R^" << Nompl << std::endl;
  std::cout << "Tangent Bundle      X = TM = M x R^{6+" << Nompl << "}" << std::endl;

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
  control_space->printSettings(std::cout);

  std::cout << std::string(80, '-') << std::endl;
}

bool KinodynamicCSpaceOMPL::isDynamic() const
{
  return true;
}

void KinodynamicCSpaceOMPL::initSpace()
{
  //###########################################################################
  // Create OMPL state space
  //   Create an SE(3) x R^n space + a R^{6+N} space
  //###########################################################################
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "only supports robots with a configuration space equal to SE(3) x R^n" << std::endl;
    throw "Unsupported robot.";
  }

  ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());

  ob::StateSpacePtr TM(std::make_shared<ob::RealVectorStateSpace>(6+Nompl));

  if(Nompl>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl));
    space = SE3 + Rn + TM;
  }else{
    space = SE3 + TM;
  }

  ob::SE3StateSpace *cspaceSE3 = space->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0);

  ob::RealVectorStateSpace *cspaceTM;
  ob::RealVectorStateSpace *cspaceRN = nullptr;

  if(Nompl>0){
    cspaceRN = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
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

  //assert(minimum.size() == 6+N);
  //assert(maximum.size() == 6+N);

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
  //###########################################################################
  // Set velocity bounds
  //###########################################################################
  std::vector<double> vMin, vMax;
  vMin = input.dqMin;
  vMax = input.dqMax;

  assert(vMin.size() == 6+Nklampt);
  assert(vMax.size() == 6+Nklampt);

  vector<double> lowTM, highTM;
  for(uint i = 0; i < 6; i++){
    lowTM.push_back(vMin.at(i));
    highTM.push_back(vMax.at(i));
  }
  for(uint i = 6; i < 6+Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    lowTM.push_back(vMin.at(idx));
    highTM.push_back(vMax.at(idx));
  }

  ob::RealVectorBounds boundsTM(6+Nompl);
  boundsTM.low = lowTM;
  boundsTM.high = highTM;
  boundsTM.check();

  OMPL_WARN("Setting manual velocity.");

  cspaceTM->setBounds(boundsTM);
}

void KinodynamicCSpaceOMPL::initControlSpace()
{
  uint NdimControl = 6 + Nompl;
  control_space = std::make_shared<oc::RealVectorControlSpace>(space, NdimControl+1);

  Vector torques = robot->torqueMax;

  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(0);
  cbounds.setHigh(0);

  cbounds.setLow(NdimControl,input.timestep_min);//propagation step size
  cbounds.setHigh(NdimControl,input.timestep_max);

  if(input.uMin.size() < 6)
  {
    OMPL_ERROR("Could not find minimum/maximum control for robot %d", GetRobotIndex());
    throw "NoControl";
  }

  for(uint i = 0; i < 6; i++){
    cbounds.setLow(i,input.uMin(i));
    cbounds.setHigh(i,input.uMax(i));
  }
  cbounds.check();
  static_pointer_cast<oc::RealVectorControlSpace>(control_space)->setBounds(cbounds);
}


ob::ScopedState<> KinodynamicCSpaceOMPL::ConfigVelocityToOMPLState(const Config &q, const Config &dq)
{
  ob::ScopedState<> qompl(space);
  ConfigVelocityToOMPLState(q, dq, qompl.get());
  return qompl;
}

void KinodynamicCSpaceOMPL::ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl)
{
  ConfigToOMPLState(q, qompl);
  ob::RealVectorStateSpace::StateType *qomplTMSpace;
  if(Nompl>0){
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }

  double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;
  for(uint i = 0; i < 6; i++){
    qomplTM[i]=dq(i);
  }
  for(uint i = 0; i < Nklampt; i++){
    int idx = klampt_to_ompl.at(i);
    if(idx<0) continue;
    else qomplTM[idx]=dq(6+i);
  }
}

void KinodynamicCSpaceOMPL::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SE3StateSpace::StateType *qomplSE3;
  ob::SO3StateSpace::StateType *qomplSO3;
  ob::RealVectorStateSpace::StateType *qomplRnSpace = nullptr;
  ob::RealVectorStateSpace::StateType *qomplTMSpace;

  //either we have
  //[SE3][RN]    OR   [ SE3]
  //[  TM   ]         [ TM ]
  //
  qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  qomplSO3 = &qomplSE3->rotation();
  if(Nompl>0){
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplRnSpace = nullptr;
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }

  qomplSE3->setXYZ(q(0),q(1),q(2));
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5), qomplSO3);

  if(Nompl>0){
    double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
    for(uint i = 0; i < Nklampt; i++){
      int idx = klampt_to_ompl.at(i);
      if(idx<0) continue;
      else qomplRn[idx]=q(6+i);
    }
  }

  double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;
  for(uint i = 0; i < (6+Nompl); i++){
    qomplTM[i]=0.0;
  }
}

Config KinodynamicCSpaceOMPL::OMPLStateToVelocity(const ob::State *qompl)
{
  const ob::RealVectorStateSpace::StateType *qomplTMState;
  if(Nompl>0){
    qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }
  Config dq;
  dq.resize(6+Nklampt);
  dq.setZero();

  for(uint i = 0; i < 6; i++){
    dq(i) = qomplTMState->values[i];
  }
  for(uint i = 0; i < Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    dq(idx) = qomplTMState->values[i];
  }
  return dq;
}

Config KinodynamicCSpaceOMPL::OMPLStateToConfig(const ob::State *qompl)
{
  if(Nompl>0){
    const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    return GeometricCSpaceOMPL::OMPLStateToConfig(qomplSE3, qomplRnState);
  }else{
    const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    Config q = GeometricCSpaceOMPL::OMPLStateToConfig(qomplSE3, NULL);
    return q;
  }
}

const oc::StatePropagatorPtr KinodynamicCSpaceOMPL::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  return std::make_shared<TangentBundleIntegrator>(si, this);
}

//#############################################################################
ob::SpaceInformationPtr KinodynamicCSpaceOMPL::SpaceInformationPtr()
{
  if(!si){
    si = std::make_shared<oc::SpaceInformation>(SpacePtr(), ControlSpacePtr());

    const ob::StateValidityCheckerPtr checker = StateValidityCheckerPtr(si);
    si->setStateValidityChecker(checker);

    oc::SpaceInformationPtr siC = static_pointer_cast<oc::SpaceInformation>(si);
    const oc::StatePropagatorPtr integrator = StatePropagatorPtr(siC);
    siC->setStatePropagator(integrator);

    siC->setMinMaxControlDuration(0.01, 0.1);
    siC->setPropagationStepSize(1);
  }
  return si;
}

Vector3 KinodynamicCSpaceOMPL::getXYZ(const ob::State *s)
{
  const ob::SE3StateSpace::StateType *qomplSE3 = s->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  double x = qomplSE3->getX();
  double y = qomplSE3->getY();
  double z = qomplSE3->getZ();
  Vector3 q(x,y,z);
  return q;
}
