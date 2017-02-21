#include <iomanip>
#include "cspace_sentinel.h"

KinodynamicCSpaceSentinelAdaptor::KinodynamicCSpaceSentinelAdaptor(CSpace *_base): 
  KinematicCSpaceAdaptor(_base), type(Euler)
{
  //this->type=RK4; 
  //this->type=Euler; 
}
//hooks for the math library integrators
struct IntegrationFunction : public DiffEqFunction
{
  IntegrationFunction(KinodynamicCSpaceSentinelAdaptor* _space,const ControlInput& _u)
    :space(_space),u(_u)
  {}

  virtual void Eval(Real t,const Vector& y,Vector& dy) {
    space->XDerivative(y,u,dy);
  }

  KinodynamicCSpaceSentinelAdaptor* space;
  const ControlInput& u;
};


void KinodynamicCSpaceSentinelAdaptor::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
{
  IntegrationFunction func(this,u);
  Real dt,h;
  int i,numSteps;
  Parameters(x0,u,dt,numSteps);
  h = dt/numSteps;
  State temp;
  p.push_back(x0);
  switch(type) {
    case Euler:
      for(i=0;i<numSteps;i++) {
        Euler_step(&func,0,h,p.back(),temp);
        p.push_back(temp);
      }
      break;
    case RK4:
      for(i=0;i<numSteps;i++) {
        RungeKutta4_step(&func,0,h,p.back(),temp);
        p.push_back(temp);
      }
      break;
    default:
      FatalError("Unknown integrator type!");
      break;
  }

}

void KinodynamicCSpaceSentinelAdaptor::SimulateEndpoint(const State& x0, const ControlInput& u,State& x1)
{
  std::vector<State> p;
  Simulate(x0,u,p);
  x1 = p.back();
}

///Return an edge planner that checks the simulation trace p for feasibility
///Typically, just return new GivenPathEdgePlanner(this,p,tolerance)
EdgePlanner* KinodynamicCSpaceSentinelAdaptor::TrajectoryChecker(const std::vector<State>& p){
  double tolerance = 1e-3;
  return new GivenPathEdgePlanner(this,p,tolerance);
}

bool KinodynamicCSpaceSentinelAdaptor::IsValidControl(const State& x,const ControlInput& u){
  const int N=6;

  double u_low_limit_ptr[N]= {0,-1,-1,1,0,0};
  double u_up_limit_ptr[N]= {0,1,1,1,0,0};
  Vector u_low_limit;
  Vector u_up_limit;
  u_low_limit.setRef(u_low_limit_ptr, N);
  u_up_limit.setRef(u_up_limit_ptr, N);

  bool valid = true;
  for(int i = 0; i < u_low_limit.size(); i++){
    valid &= (u_low_limit[i] <= u[i] && u[i] <= u_up_limit[i]);
  }
  return valid;

}

///Randomly pick a control input
void KinodynamicCSpaceSentinelAdaptor::SampleControl(const State& x,ControlInput& u){
  double ak = 1;
  u.resize(x.size());
  u.setZero();
  u(0) = 0;
  u(1) = Rand(-ak,ak);
  u(2) = Rand(-ak,ak);
  u(3) = 1;
  u(4) = 0;
  u(5) = 0;
}

void KinodynamicCSpaceSentinelAdaptor::BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u){
  SampleControl(x,u);
}

void KinodynamicCSpaceSentinelAdaptor::XDerivative(const State& x, const ControlInput& u, State& dx){

  if(!IsValidControl(x,u)){
    std::cout << "not valid control" << std::endl;
    std::cout << u << std::endl;
    exit(0);
  }

  // Lie Algebra Generators
  Matrix4 X1,X2,X3,X4,X5,X6;
  X1.setZero();
  X2.setZero();
  X3.setZero();
  X4.setZero();
  X5.setZero();
  X6.setZero();
  //##########################
  X1(1,2) = -1;
  X1(2,1) = 1;
  //##########################
  X2(0,2) = 1;
  X2(2,0) = -1;
  //##########################
  X3(0,1) = -1;
  X3(1,0) = 1;
  //##########################
  X4(0,3) = 1;
  //##########################
  X5(1,3) = 1;
  //##########################
  X6(2,3) = 1;
  
  //represent lie group element as matrix4
  RigidTransform T_x_se3;
  Matrix3 R,Rx,Ry,Rz;
  Rx.setRotateX(x(3));
  Ry.setRotateX(x(4));
  Rz.setRotateX(x(5));
  R = Rz*Ry*Rx;
  T_x_se3.setRotation(R);
  T_x_se3.setTranslation(Vector3(x(0),x(1),x(2)));

  Matrix4 x_se3;
  T_x_se3.get(x_se3);

  //#########################################################################
  //Matrix4 dx_se3 = x_se3*(X1*u(0) + X2*u(1) + X3*u(2) + X4);
  //Matrix4 dx_se3 = x_se3*(X1*u(0) + X2*u(1) + X3*u(2) + X4*u(3));
  Matrix4 dx_se3 = (X1*u(0) + X2*u(1) + X3*u(2) + X4*u(3))*x_se3;
  //#########################################################################
  std::cout << std::setprecision(2) << std::fixed;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << x_se3 << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << dx_se3 << std::endl;
  //

  //dx_se3 is a lie algebra element represented as matrix4
  // need to convert back to Vector6 element
  Matrix3 dR_m;
  EulerAngleRotation dR;
  Vector3 dT;
  dx_se3.get(dR_m,dT);

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "DR:" << dR_m << std::endl;
  std::cout << "determinant:" << dR_m.determinant() << std::endl;
  std::cout << "DT:" << dT << std::endl;
  dR.setMatrixXYZ(dR_m);


  dx.resize(x.size());
  dx.setZero();

  dx(0)=dT[0];
  dx(1)=dT[1];
  dx(2)=dT[2];
  dx(3)=dR[0];
  dx(4)=dR[1];
  dx(5)=dR[2];

  std::cout << "DX:" << dx << std::endl;

}
void KinodynamicCSpaceSentinelAdaptor::Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps){
  dt = 0.001;
  numSteps = 4;
}
