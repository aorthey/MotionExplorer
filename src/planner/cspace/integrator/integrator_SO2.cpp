#include "planner/cspace/integrator/integrator_SO2.h"
#include "planner/cspace/cspace_kinodynamic.h"

void IntegratorSO2::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{
  //(1) convert state to (q0,dq0,ddq0,u)
  //(2) integrate from (q0,dq0) to (q1,dq1);
  //(3) convert (q1,dq1) to result

  //###########################################################################
  // (1) Convert state,control to (q0,dq0,u)
  //###########################################################################
  Config q0 = cspace->OMPLStateToConfig(state);
  Config dq0 = cspace->OMPLStateToVelocity(state);
  const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  //last element is time (why not use duration?)
  uint N  = cspace->GetControlDimensionality();
  double dt = duration * ucontrol[N-1];
  double dt2 = 0.5 * dt * dt;

  Config u0 = cspace->ControlToConfig(ucontrol);

  //###########################################################################
  //(2) integrate from (q0,dq0,ddq0) to (q1,dq1);
  //###########################################################################

  Config q1(q0);
  Config dq1(dq0);

  for(int i = 0; i < q0.size(); i++){
    dq1(i) = dq0(i) + dt*u0(i);
    q1(i) = q0(i) + dt*dq0(i) + dt2*u0(i);
  }

  cspace->ConfigVelocityToOMPLState(q1, dq1, result);
}

