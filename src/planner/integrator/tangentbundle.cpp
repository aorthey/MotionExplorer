#include "planner/integrator/tangentbundle.h"
#include "planner/cspace/cspace_kinodynamic.h"

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
  // Forward Simulate SE(3) component
  //###########################################################################

  uint N  = cspace->GetControlDimensionality();
  Real dt = ucontrol[N-1];
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }

  LieGroupIntegrator integrator;

  Matrix4 q0_SE3 = integrator.StateToSE3(q0);
  Matrix4 dp0 = integrator.SE3Derivative(uSE3);
  Matrix4 q1_SE3 = integrator.Integrate(q0_SE3,dp0,dt);

  Config q1(q0);
  Config dq1(dq0);

  integrator.SE3ToState(q1, q1_SE3);

  //###########################################################################
  // Forward Simulate R^N component
  //###########################################################################
  for(uint i = 0; i < N-6-1; i++){
    dq1[i+6] = dq0[i+6] + dt*ucontrol[i+6];
  }

  cspace->ConfigVelocityToOMPLState(q1, dq1, result);

}
