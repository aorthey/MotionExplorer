#include "common.h"
#include "planner/cspace/cspace_kinodynamic_SE2.h"
#include "planner/cspace/integrator/integrator_SE2.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "ompl/base/spaces/SE2StateSpaceFullInterpolate.h"

KinodynamicCSpaceOMPLSE2::KinodynamicCSpaceOMPLSE2(RobotWorld *world_, int robot_idx):
  KinodynamicCSpaceOMPL(world_, robot_idx)
{
}

void KinodynamicCSpaceOMPLSE2::print(std::ostream& out) const
{
  ob::CompoundStateSpace *cspace = space->as<ob::CompoundStateSpace>();
  ob::SE2StateSpaceFullInterpolate *cspaceSE2 = cspace->as<ob::SE2StateSpaceFullInterpolate>(0);

  ob::RealVectorStateSpace *cspaceTM = nullptr;

  if(Nompl>0){
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2);
  }else{
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }

//################################################################################
  std::cout << "Configuration Space M = SE(2) x R^" << Nompl << std::endl;
  std::cout << "Tangent Bundle      X = TM = M x R^{3+" << Nompl << "}" << std::endl;

  const ob::RealVectorBounds bounds = cspaceSE2->getBounds();
  std::vector<double> se2min = bounds.low;
  std::vector<double> se2max = bounds.high;
  std::cout << "SE(2) bounds min     : ";
  for(uint i = 0; i < se2min.size(); i++){
    std::cout << " " << se2min.at(i);
  }
  std::cout << std::endl;

  std::cout << "SE(2) bounds max     : ";
  for(uint i = 0; i < se2max.size(); i++){
    std::cout << " " << se2max.at(i);
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

void KinodynamicCSpaceOMPLSE2::initSpace()
{
  //###########################################################################
  // Create OMPL state space
  //   Create an SE(2) x R^n space + a R^{6+N} space
  //###########################################################################
  if(!(robot->joints[0].type==RobotJoint::Floating))
  {
    std::cout << "only supports robots with a configuration space equal to SE(2) x R^n" << std::endl;
    throw "Invalid robot";
  }

  ob::StateSpacePtr SE2(std::make_shared<ob::SE2StateSpaceFullInterpolate>());

  ob::StateSpacePtr TM(std::make_shared<ob::RealVectorStateSpace>(3+Nompl));

  if(Nompl>0){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl));
    space = SE2 + Rn + TM;
  }else{
    space = SE2 + TM;
  }

  ob::SE2StateSpaceFullInterpolate *cspaceSE2 = space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpaceFullInterpolate>(0);

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

  vector<double> lowSE2;
  lowSE2.push_back(minimum.at(0));
  lowSE2.push_back(minimum.at(1));
  vector<double> highSE2;
  highSE2.push_back(maximum.at(0));
  highSE2.push_back(maximum.at(1));

  ob::RealVectorBounds boundsSE2(2);
  boundsSE2.low = lowSE2;
  boundsSE2.high = highSE2;
  boundsSE2.check();
  cspaceSE2->setBounds(boundsSE2);

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
  vMin = robot->velMin;
  vMax = robot->velMax;

  assert(vMin.size() == 6+Nklampt);
  assert(vMax.size() == 6+Nklampt);

  vector<double> lowTM, highTM;
  //Ignore all except x,y,yaw
  lowTM.push_back(vMin.at(0));
  highTM.push_back(vMax.at(0));
  lowTM.push_back(vMin.at(1));
  highTM.push_back(vMax.at(1));
  lowTM.push_back(vMin.at(3));
  highTM.push_back(vMax.at(3));

  for(uint i = 3; i < 3+Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    lowTM.push_back(vMin.at(idx));
    highTM.push_back(vMax.at(idx));
  }

  ob::RealVectorBounds boundsTM(3+Nompl);
  boundsTM.low = lowTM;
  boundsTM.high = highTM;
  boundsTM.check();
  //TODO
  boundsTM.setLow(-10);
  boundsTM.setHigh(10);
  cspaceTM->setBounds(boundsTM);

}

void KinodynamicCSpaceOMPLSE2::initControlSpace(){
  uint NdimControl = 6 + Nompl;
  control_space = std::make_shared<oc::RealVectorControlSpace>(space, NdimControl+1);

  // Vector torques = robot->torqueMax;
  // std::cout << torques << std::endl;
  // exit(0);

  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(0);
  cbounds.setHigh(0);

  cbounds.setLow(NdimControl,input.timestep_min);//propagation step size
  cbounds.setHigh(NdimControl,input.timestep_max);

  for(uint i = 0; i < 6; i++){
    cbounds.setLow(i,input.uMin(i));
    cbounds.setHigh(i,input.uMax(i));
  }
  cbounds.check();
  static_pointer_cast<oc::RealVectorControlSpace>(control_space)->setBounds(cbounds);
}


ob::ScopedState<> KinodynamicCSpaceOMPLSE2::ConfigVelocityToOMPLState(const Config &q, const Config &dq)
{
  ob::ScopedState<> qompl(space);
  ConfigVelocityToOMPLState(q, dq, qompl.get());
  return qompl;
}

void KinodynamicCSpaceOMPLSE2::ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl)
{
  ConfigToOMPLState(q, qompl);
  ob::RealVectorStateSpace::StateType *qomplTMSpace;
  if(Nompl>0){
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }

  double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;
  qomplTM[0]=dq(0);
  qomplTM[1]=dq(1);
  qomplTM[2]=dq(3);

  for(uint i = 0; i < Nklampt; i++){
    int idx = klampt_to_ompl.at(i);
    if(idx<0) continue;
    else qomplTM[idx]=dq(6+i);
  }
}

void KinodynamicCSpaceOMPLSE2::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SE2StateSpaceFullInterpolate::StateType *qomplSE2;
  ob::RealVectorStateSpace::StateType *qomplRnSpace = nullptr;
  ob::RealVectorStateSpace::StateType *qomplTMSpace;

  //either we have
  //[SE2][RN]    OR   [ SE2]
  //[  TM   ]         [ TM ]
  //
  qomplSE2 = qompl->as<ob::CompoundState>()->as<ob::SE2StateSpaceFullInterpolate::StateType>(0);
  if(Nompl>0){
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplRnSpace = nullptr;
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }

  qomplSE2->setX(q(0));
  qomplSE2->setY(q(1));
  qomplSE2->setYaw(q(3));

  if(Nompl>0){
    double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;
    for(uint i = 0; i < Nklampt; i++){
      int idx = klampt_to_ompl.at(i);
      if(idx<0) continue;
      else qomplRn[idx]=q(6+i);
    }
  }

  double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;
  for(uint i = 0; i < (3+Nompl); i++){
    qomplTM[i]=0.0;
  }
}

Config KinodynamicCSpaceOMPLSE2::OMPLStateToVelocity(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplTMState;
  if(Nompl>0){
    qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }
  Config dq;
  dq.resize(6+Nklampt);
  dq.setZero();

  dq(0) = qomplTMState->values[0];
  dq(1) = qomplTMState->values[1];
  dq(3) = qomplTMState->values[2];
  for(uint i = 0; i < Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    dq(idx) = qomplTMState->values[i];
  }
  return dq;
}

Config KinodynamicCSpaceOMPLSE2::OMPLStateToConfig(const ob::State *qompl){
  const ob::SE2StateSpaceFullInterpolate::StateType *qomplSE2 = qompl->as<ob::CompoundState>()->as<ob::SE2StateSpaceFullInterpolate::StateType>(0);

  Config q;
  q.resize(6+Nklampt);
  q.setZero();

  q(0) = qomplSE2->getX();
  q(1) = qomplSE2->getY();
  q(3) = qomplSE2->getYaw();

  if(Nompl>0){
    const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    for(uint i = 0; i < Nompl; i++){
      uint idx = ompl_to_klampt.at(i);
      q(idx) = qomplRnState->values[i];
    }
  }
  return q;
}

const oc::StatePropagatorPtr KinodynamicCSpaceOMPLSE2::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  return std::make_shared<IntegratorSE2>(si, this);
}

Vector3 KinodynamicCSpaceOMPLSE2::getXYZ(const ob::State *s)
{
  const ob::SE2StateSpace::StateType *qomplSE2 = s->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
  double x = qomplSE2->getX();
  double y = qomplSE2->getY();
  Vector3 q(x,y,0);
  return q;
}
