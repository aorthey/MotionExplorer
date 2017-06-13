#include "tangentbundle.h"
#include "ompl_space.h"

void TangentBundleIntegrator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{

  //###########################################################################
  // OMPL to Config control
  //###########################################################################
  const ob::StateSpacePtr s = si_->getStateSpace();
  Config x0 = ompl_space->OMPLStateToConfig(state);

  const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  Config use3;
  use3.resize(6);
  use3.setZero();
  use3(0) = ucontrol[0];
  use3(1) = ucontrol[1];
  use3(2) = ucontrol[2];
  use3(3) = ucontrol[3];
  use3(4) = ucontrol[4];
  use3(5) = ucontrol[5];

  uint N = 0.5*s->getDimension() - 6;
  assert( 2*N + 12 == s->getDimension());

  //###########################################################################
  // Forward Simulate SE(3) component
  //###########################################################################

  uint Nduration = N+6;
  Real dt = ucontrol[Nduration];
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }
  Real dt2 = 0.5*dt*dt;

  LieGroupIntegrator integrator;

  Matrix4 x0_SE3 = integrator.StateToSE3(x0);

  //Config dx; dx.resize(6);

  //Matrix4 dx0_SE3 = 

  Matrix4 dp0 = integrator.SE3Derivative(use3);
  Matrix4 x1_SE3 = integrator.Integrate(x0_SE3,dp0,dt);

  State x1(x0);
  integrator.SE3ToState(x1, x1_SE3);

  Config qend = x1;

  //###########################################################################
  // Forward Simulate R^N component
  //###########################################################################
  for(int i = 0; i < N; i++){
    qend[i+6] = x0[i+6] + dt*x0[i+N+6+6] + dt2*ucontrol[i+6];
    qend[i+N+6+6] = x0[i+N+6+6] + dt*ucontrol[i+6];
  }

  //###########################################################################
  // Config to OMPL
  //###########################################################################

  ob::ScopedState<> ssr = ompl_space->ConfigToOMPLState(qend);

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
