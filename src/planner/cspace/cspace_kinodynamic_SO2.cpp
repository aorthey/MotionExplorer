#include "common.h"
#include "planner/cspace/cspace_kinodynamic_SO2.h"
// #include "planner/cspace/integrator/integrator_SO2.h"
// #include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "planner/cspace/integrator/integrator_SO2.h"

KinodynamicCSpaceOMPLSO2::KinodynamicCSpaceOMPLSO2(RobotWorld *world_, int robot_idx):
  KinodynamicCSpaceOMPL(world_, robot_idx)
{
}

void KinodynamicCSpaceOMPLSO2::initSpace()
{
  //###########################################################################
  // SO(2) x R^{Nompl-1} x R^{Nompl} 
  //###########################################################################

  ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
  ob::StateSpacePtr TM(std::make_shared<ob::RealVectorStateSpace>(Nompl));

  if(Nompl>1){
    ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(Nompl-1));
    space = SO2 + Rn + TM;
  }else{
    space = SO2 + TM;
  }

  ob::RealVectorStateSpace *cspaceTM;
  ob::RealVectorStateSpace *cspaceRN = nullptr;

  if(Nompl>1){
    cspaceRN = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2);
  }else{
    cspaceTM = space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1);
  }

  //###########################################################################
  // Set Bounds on kinematic joints
  //###########################################################################
  std::vector<double> qMin = robot->qMin;
  std::vector<double> qMax = robot->qMax;

  if(Nompl>1){
    std::vector<double> lowRN;
    std::vector<double> highRN;

    //First Index is SO2 (no limits)
    for(uint k = 1; k < Nompl; k++){
      int idxk = ompl_to_klampt.at(k);
      lowRN.push_back(qMin.at(idxk));
      highRN.push_back(qMax.at(idxk));
    }

    ob::RealVectorBounds boundsRn(Nompl-1);
    boundsRn.low = lowRN;
    boundsRn.high = highRN;
    boundsRn.check();
    cspaceRN->setBounds(boundsRn);
  }

  //###########################################################################
  // Set Bounds on velocity
  //###########################################################################
  std::vector<double> dqMin = robot->velMin;
  std::vector<double> dqMax = robot->velMax;

  vector<double> lowTM;
  vector<double> highTM;

  for(uint k = 0; k < Nompl; k++){
    int idxk = ompl_to_klampt.at(k);
    lowTM.push_back(dqMin.at(idxk));
    highTM.push_back(dqMax.at(idxk));
  }

  ob::RealVectorBounds boundsTM(Nompl);
  boundsTM.low = lowTM;
  boundsTM.high = highTM;
  boundsTM.check();
  cspaceTM->setBounds(boundsTM);

}

void KinodynamicCSpaceOMPLSO2::initControlSpace(){
  uint NdimControl = 1 + Nompl;
  control_space = std::make_shared<oc::RealVectorControlSpace>(space, NdimControl+1);

  ob::RealVectorBounds cbounds(NdimControl+1);
  cbounds.setLow(0);
  cbounds.setHigh(0);

  cbounds.setLow(NdimControl,input.timestep_min);//propagation step size
  cbounds.setHigh(NdimControl,input.timestep_max);

  for(uint k = 0; k < Nompl; k++)
  {
    int idx = ompl_to_klampt.at(k);
    cbounds.setLow(k, input.uMin(idx));
    cbounds.setHigh(k, input.uMax(idx));
  }

  cbounds.check();
  static_pointer_cast<oc::RealVectorControlSpace>(control_space)->setBounds(cbounds);
}

Config KinodynamicCSpaceOMPLSO2::ControlToConfig(const double* control)
{
  Config u; u.resize(6 + Nklampt); u.setZero();
  for(uint k = 0; k < Nompl; k++)
  {
    int idx = ompl_to_klampt.at(k);
    u(idx) = control[k];
  }
  return u;
}

void KinodynamicCSpaceOMPLSO2::ConfigVelocityToOMPLState(const Config &q, const Config &dq, ob::State *qompl)
{
  ConfigToOMPLState(q, qompl);

  ob::RealVectorStateSpace::StateType *qomplTMSpace;
  if(Nompl>1){
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }

  // double* qomplTM = static_cast<ob::RealVectorStateSpace::StateType*>(qomplTMSpace)->values;

  for(uint i = 0; i < Nklampt; i++){
    int idx = klampt_to_ompl.at(i);
    if(idx<0) continue;
    qomplTMSpace->values[idx]=dq(6+i);
  }
}

void KinodynamicCSpaceOMPLSO2::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SO2StateSpace::StateType *qomplSO2;
  ob::RealVectorStateSpace::StateType *qomplRnSpace = nullptr;

  qomplSO2 = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
  if(Nompl>1)
  {
    qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    for(uint k = 6; k < 6 + Nklampt; k++)
    {
      int idx = klampt_to_ompl.at(k);
      if(idx<0) continue;
      if(idx == 0) 
      {
        qomplSO2->value = q(k);
      }else{
        qomplRnSpace->values[idx] = q(k); 
      }
    }
  }else{
    uint idx = ompl_to_klampt.at(0);
    qomplSO2->value = q(idx);
  }

}

Config KinodynamicCSpaceOMPLSO2::OMPLStateToVelocity(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplTMState;
  if(Nompl>1){
    qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  }else{
    qomplTMState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  }

  Config dq; dq.resize(6+Nklampt); dq.setZero();

  for(uint i = 0; i < Nompl; i++){
    int idx = ompl_to_klampt.at(i);
    dq(idx) = qomplTMState->values[i];
  }
  return dq;
}

Config KinodynamicCSpaceOMPLSO2::OMPLStateToConfig(const ob::State *qompl)
{
  const ob::SO2StateSpace::StateType *qomplSO2 = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

  Config q;
  q.resize(6+Nklampt);
  q.setZero();

  uint idx = ompl_to_klampt.at(0);
  q(idx) = qomplSO2->value;

  if(Nompl>1)
  {
    const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    for(uint i = 1; i < Nompl; i++)
    {
      uint idx = ompl_to_klampt.at(i);
      q(idx) = qomplRnState->values[i];
    }
  }
  return q;
}

const oc::StatePropagatorPtr KinodynamicCSpaceOMPLSO2::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  return std::make_shared<IntegratorSO2>(si, this);
}

Vector3 KinodynamicCSpaceOMPLSO2::getXYZ(const ob::State *s)
{
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
