#include "planner/cspace/integrator/integrator_SE2.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/cspace/integrator/liegroupintegrator.h"

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
void IntegratorSE2::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
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
  uSE3(2) = 0;
  uSE3(3) = ucontrol[3];
  uSE3(4) = 0;
  uSE3(5) = 0;

  //std::cout << uSE3 << std::endl;

  //###########################################################################
  //(2) integrate from (q0,dq0) to (q1,dq1);
  //###########################################################################

  uint N  = cspace->GetControlDimensionality();
  Real dt = ucontrol[N-1];
  //Real dt2 = 0.5*dt*dt;
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

  // for(uint i = 0; i < N-3-1; i++){
  //   dq1[i+6] = dq0[i+6] + dt*ucontrol[i+3];
  //   q1[i+6] = q0[i+6] + dt*dq0[i+6] + dt2*ucontrol[i+3];
  // }
  //###########################################################################
  //(3) convert (q1,dq1) to result
  //###########################################################################

  cspace->ConfigVelocityToOMPLState(q1, dq1, result);
}

