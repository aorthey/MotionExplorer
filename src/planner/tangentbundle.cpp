#include "tangentbundle.h"
#include "ompl_space.h"

void TangentBundleIntegrator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{

  //
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

  //###########################################################################
  // OMPL to Config control
  //###########################################################################
  const ob::StateSpacePtr s = si_->getStateSpace();
  Config q0 = ompl_space->OMPLStateToConfig(state);

  const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  Config R6control;
  R6control.resize(6);
  R6control.setZero();
  R6control(0) = ucontrol[0];
  R6control(1) = ucontrol[1];
  R6control(2) = ucontrol[2];
  R6control(3) = ucontrol[3];
  R6control(4) = ucontrol[4];
  R6control(5) = ucontrol[5];

  uint N = 0.5*s->getDimension() - 6;
  assert( 2*N + 12 == s->getDimension());

  //###########################################################################
  // extract step size parameter
  //###########################################################################

  uint Nduration = N+6;
  Real dt = ucontrol[Nduration];
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }
  Real dt2 = 0.5*dt*dt;

  //###########################################################################
  // Forward Simulate SE(3) component
  //###########################################################################
  LieGroupIntegrator integrator;

  Config x0; x0.resize(6);
  Config dx0; dx0.resize(6);
  for(int i = 0; i < 6; i++){
    x0(i) = q0(i);
    dx0(i) = q0(i+6+N);
  }

  Matrix4 x0_SE3 = integrator.StateToSE3(x0);

  Matrix4 dx0_SE3 = integrator.SE3Derivative(dx0);

  Matrix4 ddp = integrator.SE3Derivative(R6control);

  Matrix4 dp = ddp*dt*0.5 + dx0_SE3;

  Matrix4 x1_SE3 = integrator.Integrate(x0_SE3,dp,dt);

  State x1;x1.resize(6);
  integrator.SE3ToState(x1, x1_SE3);

  State dx1 = R6control*dt + dx0;
  //integrator.SE3ToState(dx1, dx1_SE3);

  //State x1(x0); integrator.SE3ToState(x1, x1_SE3);
  //Config qend = x1;

  Config q1; q1.resize(12+2*N); q1.setZero();
  for(int i = 0; i < 6; i++){
    q1(i) = x1(i);
    q1(i+6+N) = dx1(i);
  }

  //###########################################################################
  // Forward Simulate R^N component
  //###########################################################################
  for(int i = 0; i < N; i++){
    q1[i+6] = q0[i+6] + dt*q0[i+N+6+6] + dt2*ucontrol[i+6];
    q1[i+N+6+6] = q0[i+N+6+6] + dt*ucontrol[i+6];
  }

  //###########################################################################
  // Config to OMPL
  //###########################################################################

  ob::ScopedState<> ssr = ompl_space->ConfigToOMPLState(q1);

  ob::SE3StateSpace::StateType *ssrSE3 = ssr->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *ssrSO3 = &ssrSE3->rotation();
  ob::RealVectorStateSpace::StateType *ssrRn = ssr->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  ob::RealVectorStateSpace::StateType *ssrTM = ssr->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

  ob::SE3StateSpace::StateType *resultSE3 = result->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *resultSO3 = &resultSE3->rotation();
  ob::RealVectorStateSpace::StateType *resultRn = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  ob::RealVectorStateSpace::StateType *resultTM = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

  resultSE3->setXYZ(ssrSE3->getX(),ssrSE3->getY(),ssrSE3->getZ());
  resultSO3->x = ssrSO3->x;
  resultSO3->y = ssrSO3->y;
  resultSO3->z = ssrSO3->z;
  resultSO3->w = ssrSO3->w;

  //###########################################################################
  // R^N Control
  //###########################################################################
  for(int i = 0; i < N; i++){
    resultRn->values[i] = ssrRn->values[i];
  }
  for(int i = 0; i < N+6; i++){
    resultTM->values[i] = ssrTM->values[i];
  }

}

TangentBundleOMPLValidityChecker::TangentBundleOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace* space, CSpaceOMPL* ompl_space_):
  ob::StateValidityChecker(si),cspace_(space),ompl_space(ompl_space_)
{
}

bool TangentBundleOMPLValidityChecker::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = ompl_space->OMPLStateToConfig(state);
  return cspace_->IsFeasible(q) && si_->satisfiesBounds(state);
}
