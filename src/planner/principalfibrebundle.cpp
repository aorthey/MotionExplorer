#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include "principalfibrebundle.h"

using namespace Math3D;

PrincipalFibreBundleAdaptor::PrincipalFibreBundleAdaptor(CSpace *_base): 
  KinematicCSpaceAdaptor(_base)
{
}

void PrincipalFibreBundleAdaptor::SimulateEndpoint(const State& x0, const ControlInput& u,State& x1)
{
  exit(0);
  std::vector<State> p;
  Simulate(x0,u,p);
  x1 = p.back();
}

///Return an edge planner that checks the simulation trace p for feasibility
///Typically, just return new GivenPathEdgePlanner(this,p,tolerance)
EdgePlanner* PrincipalFibreBundleAdaptor::TrajectoryChecker(const std::vector<State>& p){
  exit(0);
  double tolerance = 1e-3;
  return new GivenPathEdgePlanner(this,p,tolerance);
  //return new TrueEdgePlanner(this,p.front(),p.back());
}

bool PrincipalFibreBundleAdaptor::IsValidControl(const State& x,const ControlInput& u){
  exit(0);
  return true;
  //const int N=6;

  //double u_low_limit_ptr[N]= {0,-1,-1,1,0,0};
  //double u_up_limit_ptr[N]= {0,1,1,1,0,0};
  //Vector u_low_limit;
  //Vector u_up_limit;
  //u_low_limit.setRef(u_low_limit_ptr, N);
  //u_up_limit.setRef(u_up_limit_ptr, N);

  //bool valid = true;
  //for(int i = 0; i < u_low_limit.size(); i++){
  //  valid &= (u_low_limit[i] <= u[i] && u[i] <= u_up_limit[i]);
  //}
  //if(!valid){
  //  std::cout << "Non valid control!" << std::endl;
  //  std::cout << u << std::endl;
  //  exit(0);
  //}
  //return valid;

}

void PrincipalFibreBundleAdaptor::BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u){
  BiasedSampleControl(x1,xDest,u);
  //BiasedSampleControl(xDest,x1,u);
}

Real PrincipalFibreBundleAdaptor::Distance(const Config& x, const Config& y) { 
  //return base->Distance(x,y); 
 // Config xpos;xpos.resize(3);xpos(0)=x(0);xpos(1)=x(1);xpos(2)=x(2);
 // Config ypos;ypos.resize(3);ypos(0)=y(0);ypos(1)=y(1);ypos(2)=y(2);
 // return base->Distance(xpos,ypos); 

  exit(0);
  RigidTransform Ta,Tb;
  ConfigToTransform(x,Ta);
  ConfigToTransform(y,Tb);

  ////####

  Vector3 e1(1,0,0);
  Vector3 ydir = Tb.R*e1;
  Vector3 ypos = Tb.t;
  Vector3 xpos = Ta.t;
  //Vector3 fpos = ypos - ydir*0.3;

  Vector3 xydir = xpos - ypos;
  double r=dot(xydir,ydir);

  Vector3 xline = xydir - r*ydir;

  double df = xline.norm();
  //exit(0);
  //std::cout << r << " " << df << std::endl;
  //float theta = M_PI/6;
  //if (acos(dot(X-M, N)/(norm(X-M)*norm(N)) <= theta) doSomething();
  //double d = ydir.dot(xdir);
  ////####

  Real d = Ta.t.distance(Tb.t);
  Matrix3 Rrel;
  Rrel.mulTransposeB(Ta.R,Tb.R);
  AngleAxisRotation aa;
  aa.setMatrix(Rrel);
  double wt = 1;
  double wf = 0.0;
  double wr = 0.1;
  d = Sqrt(d*d*wt + df*df*wf + aa.angle*aa.angle*wr);

  return d;
}

///Randomly pick a control input
void PrincipalFibreBundleAdaptor::SampleControl(const State& x,ControlInput& u){
  double ak = 1;
  u.resize(x.size());
  u.setZero();
  u(0) = 0;
  u(1) = Rand(-ak,+ak);
  u(2) = Rand(-ak,+ak);
  u(3) = 1;
  u(4) = 0;
  u(5) = 0;
  //T
  //u(6) = Rand(0.01,0.2);
  u(6) = Rand(0.01,0.1);
  exit(0);
}
void PrincipalFibreBundleAdaptor::BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u){
  int numSamples = 1e2;
  Real closest=Inf;

  for(int i=0;i<numSamples;i++) {
    State x2,xtemp;
    ControlInput temp;
    SampleControl(x,temp);
    SimulateEndpoint(x,temp,x2);
    Real dist = Distance(x2,xGoal);
    if(dist < closest) {
      closest = dist;
      u = temp;
    }
  }
  exit(0);
}

void PrincipalFibreBundleIntegrator::propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const 
{

  //###########################################################################
  // OMPL to Config control
  //###########################################################################
  const ob::StateSpacePtr s = si_->getStateSpace();
  Config x0 = OMPLStateToConfig(state, s);

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

  uint N = s->getDimension() - 6;

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
  for(int i = 0; i < N; i++){
    qend[i+6] = x0[i+6] + dt*ucontrol[i+6];
  }

  //###########################################################################
  // Config to OMPL
  //###########################################################################

  ob::ScopedState<> ssr = ConfigToOMPLState(qend, s);

  ob::SE3StateSpace::StateType *ssrSE3 = ssr->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *ssrSO3 = &ssrSE3->rotation();
  ob::RealVectorStateSpace::StateType *ssrRn = ssr->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

  ob::SE3StateSpace::StateType *resultSE3 = result->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *resultSO3 = &resultSE3->rotation();
  ob::RealVectorStateSpace::StateType *resultRn = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

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

}

PrincipalFibreBundleOMPLValidityChecker::PrincipalFibreBundleOMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpace* space):
  ob::StateValidityChecker(si),cspace_(space)
{
}

bool PrincipalFibreBundleOMPLValidityChecker::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = OMPLStateToConfig(state, ssp);
  //Robot *robot = _space->robot;
  //if(!robot->InJointLimits(q)) {
    //return false;
  //}
  //return _space->CheckCollisionFree();

  //PropertyMap pmap;
  //_space->Properties(pmap);

  return cspace_->IsFeasible(q) && si_->satisfiesBounds(state);
}



