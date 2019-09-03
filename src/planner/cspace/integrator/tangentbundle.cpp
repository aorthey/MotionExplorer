#include "planner/cspace/integrator/tangentbundle.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/cspace/integrator/liegroupintegrator.h"

Matrix3 GetTotalInertiaAtPoint(const Robot *robot, const Vector3 &p)
{
  Matrix3 I,temp,ccross;
  I.setZero();
  Vector3 ci;
  for(size_t i=0;i<robot->links.size();i++) {
    robot->links[i].GetWorldInertia(temp);
    I += temp;
    //get the inertia matrix associated with the center of mass
    robot->links[i].T_World.mul(robot->links[i].com,ci);
    ci -= p;
    Real x,y,z;
    ci.get(x,y,z);
    ccross(0,0)=y*y+z*z; ccross(0,1)=-x*y; ccross(0,2)=-x*z;
    ccross(1,0)=-x*y; ccross(1,1)=x*x+z*z; ccross(1,2)=-y*z;
    ccross(2,0)=-x*z; ccross(2,1)=-y*z; ccross(2,2)=x*x+y*y;
    ccross.inplaceMul(robot->links[i].mass);
    I += ccross;
  }
  return I;
}

// Configuration space is Q = G x M whereby G is the position space and M the
// shape space. We write \hat{q} = (x,r) \in Q for an element in Q.
//
// State Space is TQ with an element represented as q = (\hat{q},\dot{\hat{q}}) or
// ((x,r),(\dot{x},\dot{r}))
//
// The local chart on which q resides has dimensionality 6 + N + 6 + N = 12+2N
// with G=SE(3), M=R^N, TG \times TM = R^{6+N}
//
// Control is applied on the tangent space TTQ of the tangent bundle TQ.
// The control dimensionality is R^{6+N} \times R whereby the last dimension
// is the step size parameter (some algorithms are sensitive to the stepsize,
// so it is advantageous to change it, adapt it on the fly or even include it
// into some optimization routine)
//
void TangentBundleIntegrator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{
  //(1) convert state to (q0,dq0,ddq0,u)
  //(2) integrate from (q0,dq0) to (q1,dq1);
  //(3) convert (q1,dq1) to result

  //###########################################################################
  // (1) Convert state,control to (q0,dq0,ddq0,u)
  //###########################################################################
  Config q0 = cspace->OMPLStateToConfig(state);
  Config dq0 = cspace->OMPLStateToVelocity(state);

  const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  Config uSE3;
  uSE3.resize(6);
  uSE3.setZero();
  uSE3(0) = ucontrol[0];
  uSE3(1) = ucontrol[1];
  uSE3(2) = ucontrol[2];
  uSE3(3) = ucontrol[3];
  uSE3(4) = ucontrol[4];
  uSE3(5) = ucontrol[5];

  //###########################################################################
  //(2) integrate from (q0,dq0) to (q1,dq1);
  //###########################################################################

  uint N  = cspace->GetControlDimensionality();
  Real dt = ucontrol[N-1];
  Real dt2 = 0.5*dt*dt;
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }

  Config q1(q0);
  Config dq1(dq0);

  for(int i = 0; i < uSE3.size(); i++){
    dq1(i) = dq0(i) + dt*uSE3(i);
  }

  LieGroupIntegrator integrator;
  Matrix4 q0_SE3 = integrator.StateToSE3(q0);
  Matrix4 dtmp = integrator.SE3Derivative(dq1);
  Matrix4 q1_SE3 = integrator.Integrate(q0_SE3,dtmp,dt);

  integrator.SE3ToState(q1, q1_SE3);

  for(uint i = 0; i < N-6-1; i++){
    dq1[i+6] = dq0[i+6] + dt*ucontrol[i+6];
    q1[i+6] = q0[i+6] + dt*dq0[i+6] + dt2*ucontrol[i+6];
  }
  //###########################################################################
  //(3) convert (q1,dq1) to result
  //###########################################################################

  cspace->ConfigVelocityToOMPLState(q1, dq1, result);
}

void TangentBundleIntegrator::propagate_deprecated(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{
  //###########################################################################
  // (1) Convert state,control to input=(q0,dq0,u)
  //###########################################################################
  // state is element of tangentbundle TM. 
  Config q0 = cspace->OMPLStateToConfig(state);
  Config dq0 = cspace->OMPLStateToVelocity(state);

  const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  uint Nctrl  = cspace->GetControlDimensionality();

  //last entry is timestep
  Config u; u.resize(Nctrl-1); u.setZero();
  for(uint k = 0; k < Nctrl-1; k++) u(k) = ucontrol[k];
  double dt = ucontrol[Nctrl-1];
  //double dt2 = 0.5*dt*dt;
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }

  //###########################################################################
  // (2) extract forces Fext at (q0,dq0) from simulator
  //###########################################################################

  Robot *robot = cspace->GetRobotPtr();
  robot->q = q0;
  robot->dq = dq0;
  robot->UpdateConfig(q0);
  robot->UpdateDynamics();

  uint Nklampt = cspace->GetKlamptDimensionality();

  Vector fext; fext.resize(Nklampt); fext.setZero();
  //fext = S^T * torque, whereby S is the selection matrix
  for(uint k = 6; k < Nklampt; k++){
    fext(k) = ucontrol[k];
  }

  //###########################################################################
  // (3) control uand forces are assumed to be instantaneuous at configuration
  // (q0,dq0). 
  //###########################################################################

}

  // //###########################################################################
  // //###########################################################################

  // Real dt = ucontrol[N-1];
  // if(dt<0){
  //   std::cout << "propagation step size is negative:"<<dt << std::endl;
  //   exit(0);
  // }

  // LieGroupIntegrator integrator;

  // Matrix4 q0_SE3 = integrator.StateToSE3(q0);
  // Matrix4 dp0 = integrator.SE3Derivative(uSE3);
  // Matrix4 q1_SE3 = integrator.Integrate(q0_SE3,dp0,dt);

  // Config q1(q0);
  // Config dq1(dq0);

  // integrator.SE3ToState(q1, q1_SE3);

  // //###########################################################################
  // // Forward Simulate R^N component
  // //###########################################################################
  // for(uint i = 0; i < N-6-1; i++){
  //   dq1[i+6] = dq0[i+6] + dt*ucontrol[i+6];
  // }

  // cspace->ConfigVelocityToOMPLState(q1, dq1, result);


  // Vector q0,dq0;q0.resize(6+N);dq0.resize(6+N);
  // for(uint i = 0; i < 6+N; i++){
  //   q0(i) = qstate(i);
  //   dq0(i) = qstate(i+6+N);
  // }

  // robot->dq = dq0;
  // robot->UpdateConfig(q0);
  // robot->UpdateDynamics();

  // uint lidx = 5;
  // RobotLink3D *link  = &robot->links.at(lidx);
  // Vector3 com = link->com;
  // Matrix3 R = link->T_World.R;
  // //Frame3D Tw = link->T_World;


  // Config q1; q1.resize(12+2*N); q1.setZero();
  // Config dq1;dq1.resize(6+N); dq1.setZero();
  // Config ddq0; ddq0.resize(6+N); ddq0.setZero();
  // //###########################################################################
  // // update based on real dynamics
  // //###########################################################################
  // Vector fext; fext.resize(6+N);
  // fext.setZero();

  // //fext = S^T * torque, whereby S is the selection matrix
  // for(uint k = 6; k < N+6; k++){
  //   fext(k) = ucontrol[k];
  // }

  // Vector3 force,force_tmp(ucontrol[0],ucontrol[1],ucontrol[2]);
  // //Vector3 torque_tmp(ucontrol[5],ucontrol[4],ucontrol[3]);
  // Vector3 torque(ucontrol[5],ucontrol[4],ucontrol[3]);

  // R.mul(force_tmp, force);
  // //R.mul(torque_tmp, torque);

  // // Vector Fq;
  // // robot->GetWrenchTorques(tmp, force, 5, Fq);
  // // fext += Fq;

  // Matrix3 inertia = GetTotalInertiaAtPoint(robot, link->T_World*com);
  // //Matrix3 inertia = robot->GetTotalInertia();

  // Matrix3 inertia_inv;
  // inertia.getInverse(inertia_inv);

  // robot->CalcAcceleration(ddq0, fext);

  // Vector Cdq;
  // robot->GetCoriolisForces(Cdq);

  // for(int i = 0; i < 3; i++){
  //   force[i]=force[i]-Cdq(i);
  //   torque[i]=torque[i]-Cdq(i+3);
  // }

  // Vector3 ddqTorque,ddqForce;
  // inertia_inv.mul(torque, ddqTorque);

  // ddqForce = force/robot->GetTotalMass();
  // //std::cout << inertia_inv << std::endl;
  // //inertia_inv.mul(force, ddqForce);


  // for(int i = 0; i < 3; i++){
  //   Vector3 w = robot->links[i].w;
  //   Vector3 wf = w*ddqForce[i];
  //   ddq0(0) += wf[0];
  //   ddq0(1) += wf[1];
  //   ddq0(2) += wf[2];
  // }
  // for(int i = 3; i < 6; i++){
  //   Vector3 w = robot->links[i].w;
  //   Vector3 wt = w*ddqTorque[i-3];
  //   ddq0(3) += wt[0];
  //   ddq0(4) += wt[1];
  //   ddq0(5) += wt[2];
  // }

// //################################################################################/
// // ddq0 seems to be correct if we integrate the usual way. However, once we go
// // over the rotation limits, it becomes wrong. Should use liegroupintegrator,
// // but there seems to be some bug
// //################################################################################/
  // //
  // //*
  //  LieGroupIntegrator integrator;

  //  Config x0; x0.resize(6);
  //  Config dx0; dx0.resize(6);
  //  Config ddx0; ddx0.resize(6);
  //  for(int i = 0; i < 6; i++){
  //    x0(i) = q0(i);
  //    dx0(i) = dq0(i);
  //    ddx0(i) = ddq0(i);
  //  }

  //  Vector3 dxf,dxt,ddxf,ddxt;
  //  Vector3 dxf_tmp,dxt_tmp,ddxf_tmp,ddxt_tmp;

  //  for(int i = 0; i < 3; i++) ddxf_tmp[i]=ddx0(i);
  //  R.mulTranspose(ddxf_tmp, ddxf);
  //  for(int i = 0; i < 3; i++) ddx0(i)=ddxf[i];

  //  for(int i = 0; i < 3; i++) dxf_tmp[i]=dx0(i);
  //  R.mulTranspose(dxf_tmp, dxf);
  //  for(int i = 0; i < 3; i++) dx0(i)=dxf[i];

  //  Matrix4 x0_SE3 = integrator.StateToSE3(x0);

  //  Matrix4 dx0_SE3 = integrator.SE3Derivative(dx0);

  //  Matrix4 ddp = integrator.SE3Derivative(ddx0);

  //  Matrix4 dp = ddp*dt*0.5 + dx0_SE3;

  //  Matrix4 x1_SE3 = integrator.Integrate(x0_SE3,dp,dt);

  //  State x1;x1.resize(6);
  //  integrator.SE3ToState(x1, x1_SE3);

  //  State dx1 = ddx0*dt + dx0;

  //  for(int i = 0; i < 3; i++) dxf_tmp[i]=dx1(i);
  //  R.mul(dxf_tmp, dxf);
  //  for(int i = 0; i < 3; i++) dx1(i)=dxf[i];

  //  for(int i = 0; i < 6; i++){
  //    q1(i) = x1(i);
  //    q1(i+6+N) = dx1(i);
  //    dq1(i) = dx1(i);
  //  }

// //   static uint xx = 0;
// //   if(xx++ < 5){
// //     std::cout << q0 << std::endl;
// //     std::cout << dq0 << std::endl;
// //     std::cout << R << std::endl;
// //     std::cout << ddqForce << std::endl;
// //     std::cout << ddqTorque << std::endl;
// //     std::cout << "dx1  :" << dx1 << std::endl;
  // }

  // Vector3 ddqTorque,ddqForce;
  // inertia_inv.mul(torque, ddqTorque);

  // ddqForce = force/robot->GetTotalMass();
  // //std::cout << inertia_inv << std::endl;
  // //inertia_inv.mul(force, ddqForce);


  // for(int i = 0; i < 3; i++){
  //   Vector3 w = robot->links[i].w;
  //   Vector3 wf = w*ddqForce[i];
  //   ddq0(0) += wf[0];
  //   ddq0(1) += wf[1];
  //   ddq0(2) += wf[2];
  // }
  // for(int i = 3; i < 6; i++){
  //   Vector3 w = robot->links[i].w;
  //   Vector3 wt = w*ddqTorque[i-3];
  //   ddq0(3) += wt[0];
  //   ddq0(4) += wt[1];
  //   ddq0(5) += wt[2];
  // }

