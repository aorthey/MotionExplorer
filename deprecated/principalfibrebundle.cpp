//#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include "planner/integrator/principalfibrebundle.h"
#include "planner/cspace/cspace_kinodynamic.h"

using namespace Math3D;

void PrincipalFibreBundleIntegrator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{
  // const ob::SE3StateSpace::StateType *ssrSE3 = state->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  // const ob::SO3StateSpace::StateType *ssrSO3 = &ssrSE3->rotation();

  // ob::SE3StateSpace::StateType *resultSE3 = result->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  // ob::SO3StateSpace::StateType *resultSO3 = &resultSE3->rotation();

  // ob::RealVectorStateSpace::StateType *resultTM = nullptr;
  // uint Nompl = cspace->GetControlDimensionality() - 6 -1;

  // std::cout << Nompl << std::endl;
  // if(Nompl>0){
  //   resultTM = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
  // }else{
  //   resultTM = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  // }

  // for(uint k = 0; k < cspace->GetControlDimensionality(); k++){
  //   resultTM->values[k] = 0.0;
  // }

  // double x = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
  // resultSE3->setXYZ(ssrSE3->getX()+x,ssrSE3->getY(),ssrSE3->getZ());
  // resultSO3->x = ssrSO3->x;
  // resultSO3->y = ssrSO3->y;
  // resultSO3->z = ssrSO3->z;
  // resultSO3->w = ssrSO3->w;

  // //si_->getStateSpace()->enforceBounds(result);
  // std::cout << "PROPAGATE" << std::endl;
  // si_->printState(state);
  // si_->printState(result);
  // return;


  //###########################################################################
  // OMPL to Config control
  //###########################################################################
  const ob::StateSpacePtr space = si_->getStateSpace();
  Config x0 = cspace->OMPLStateToConfig(state);

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

  uint N = space->getDimension() - 6;
  std::cout << N << std::endl;

  //###########################################################################
  // Forward Simulate SE(3) component
  //###########################################################################

  uint Nduration = N+6;
  Real dt = ucontrol[Nduration];
  if(dt<0){
    std::cout << "propagation step size is negative:"<<dt << std::endl;
    exit(0);
  }

  LieGroupIntegrator integrator;

  Matrix4 x0_SE3 = integrator.StateToSE3(x0);
  Matrix4 dp0 = integrator.SE3Derivative(uSE3);
  Matrix4 x1_SE3 = integrator.Integrate(x0_SE3,dp0,dt);

  State x1(x0);
  integrator.SE3ToState(x1, x1_SE3);

  Config qend = x1;

  //###########################################################################
  // Forward Simulate R^N component
  //###########################################################################
  for(uint i = 0; i < N; i++){
    qend[i+6] = x0[i+6] + dt*ucontrol[i+6];
  }

  //###########################################################################
  // Config to OMPL
  //###########################################################################

  qend = x0;
  std::cout << "qend= " << qend << std::endl;
  ob::ScopedState<> ssr = cspace->ConfigVelocityToOMPLState(qend, qend);

  ob::SE3StateSpace::StateType *ssrSE3;
  ob::SE3StateSpace::StateType *resultSE3;

  if(N>0){
    ssrSE3 = ssr->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    resultSE3 = result->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  }else{
    ssrSE3 = ssr->as<ob::SE3StateSpace::StateType>();
    resultSE3 = result->as<ob::SE3StateSpace::StateType>();
  }

  ob::SO3StateSpace::StateType *ssrSO3 = &ssrSE3->rotation();
  ob::SO3StateSpace::StateType *resultSO3 = &resultSE3->rotation();
  resultSE3->setXYZ(ssrSE3->getX(),ssrSE3->getY(),ssrSE3->getZ());
  resultSO3->x = ssrSO3->x;
  resultSO3->y = ssrSO3->y;
  resultSO3->z = ssrSO3->z;
  resultSO3->w = ssrSO3->w;

  //###########################################################################
  // R^N Control
  //###########################################################################
  uint Nompl = cspace->GetControlDimensionality() - 6 -1;
  if(Nompl>0){
    ob::RealVectorStateSpace::StateType *ssrRn = ssr->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    ob::RealVectorStateSpace::StateType *resultRn = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    for(uint i = 0; i < N; i++){
      resultRn->values[i] = ssrRn->values[i];
    }
  }

}
