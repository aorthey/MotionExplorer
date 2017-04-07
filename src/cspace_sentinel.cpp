#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include "cspace_sentinel.h"

KinodynamicCSpaceSentinelAdaptor::KinodynamicCSpaceSentinelAdaptor(CSpace *_base): 
  KinematicCSpaceAdaptor(_base), type(Euler)
{
}

void KinodynamicCSpaceSentinelAdaptor::Euler_step(std::vector<Matrix4>& p, const Matrix4& dp0, double h)
{  
  Matrix4 p0 = p.back();
  Matrix4 pnext = ForwardSimulate(p0,dp0,h);
  p.push_back(pnext);
}
void KinodynamicCSpaceSentinelAdaptor::RK4_step(std::vector<Matrix4>& p, const Matrix4& dp0, double h)
{  
  Matrix4 p0 = p.back();

  Matrix4 k1 = dp0;
  Matrix4 k2 = ForwardSimulate(p0+k1*h/2,dp0,0);
  Matrix4 k3 = ForwardSimulate(p0+k2*h/2,dp0,0);
  Matrix4 k4 = ForwardSimulate(p0+k3*h/2,dp0,0);
  Matrix4 pnext = p0 + (k1+k2*2+k3*2+k4)*h/6.0;
  p.push_back(pnext);
}

Matrix4 KinodynamicCSpaceSentinelAdaptor::ForwardSimulate(const Matrix4& p0, const Matrix4& dp0, double h)
{
  Matrix4 tmp = MatrixExponential(dp0*h);
  Matrix4 pnext = p0*tmp;
  return pnext;
}
void KinodynamicCSpaceSentinelAdaptor::RungeKutta4_step(std::vector<Matrix4>& p, const Matrix4& dp0, double h)
{  
  throw("NYI");
  Matrix4 p0 = p.back();
  //Matrix4 pnext = ForwardSimulate(p0,dp0,h);
  //p.push_back(pnext);
  Matrix4 k1,k2,k3,k4;
  k1 = ForwardSimulate(p0, dp0,           0);
  k2 = ForwardSimulate(p0, dp0+k1*h/2.0,  h/2.0);
  k3 = ForwardSimulate(p0, dp0+k2*h/2.0,  h/2.0);
  k4 = ForwardSimulate(p0, dp0+k3*h,      h);

  Matrix4 m = (k1+k2*2+k3*2+k4)/6.0;

  Matrix4 pnext = ForwardSimulate(p0, m, h);
  p.push_back(pnext);
}
//Real RungeKutta4_step(RealFunction2* f, Real t0, Real h, Real w)
//{
//  Real k1,k2,k3,k4;
//  k1 = h*(*f)(t0,w);
//  k2 = h*(*f)(t0+h*Half, w+k1*Half);
//  k3 = h*(*f)(t0+h*Half, w+k2*Half);
//  k4 = h*(*f)(t0+h, w+k3);
//  return w + (k1 + Two*k2 + Two*k3 + k4)/6.0;
//}


void KinodynamicCSpaceSentinelAdaptor::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
{
  //State x0 lies on local chart represented by R^6. We will convert x0
  //to a R^4x4 matrix representing the same SE(3) element. Using the matrix
  //representation allows easier computation of the forward dynamics. Once we
  //are done, we reconvert the matrix representation to an element in vector
  //representation in R^6 on the local chart.
  //
  // p is the path segment containing (numsteps) states along the forward simulation
  // dt is the total timestep size
  // numsteps is the number intermediate steps
  // h = dt/numsteps is the intermediate timestep size

  std::cout << std::setprecision(2) << std::fixed;
  Real dt = u(6);
  int numSteps = 1;
  Real h = dt/numSteps;

  Matrix4 x0_SE3 = StateToSE3(x0);
  Matrix4 dp0 = this->SE3Derivative(u);

  std::vector<Matrix4> p_SE3;
  p_SE3.push_back(x0_SE3);

  for(int i=0;i<numSteps;i++) {
    Euler_step(p_SE3, dp0, h);
    //RK4_step(p_SE3, dp0, h);
    //RungeKutta4_step(p_SE3, dp0, h);
  }

  p.push_back(x0);
  for(int i = 1; i < p_SE3.size(); i++){
    State pi(x0);
    SE3ToState(pi, p_SE3.at(i));
    p.push_back(pi);
  }

}


Matrix4 KinodynamicCSpaceSentinelAdaptor::MatrixExponential(const Matrix4& x)
{
  Eigen::MatrixXd A(4,4);
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      A(i,j) = x(i,j);
    }
  }

  Eigen::MatrixXd Aexp = A.exp();
  Matrix4 result;
  //std::cout << "The matrix exponential of A is:\n" << Aexp << "\n\n";
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      result(i,j) = Aexp(i,j);
    }
  }
  return result;
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
  //return new TrueEdgePlanner(this,p.front(),p.back());
}

bool KinodynamicCSpaceSentinelAdaptor::IsValidControl(const State& x,const ControlInput& u){
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

void KinodynamicCSpaceSentinelAdaptor::BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u){
  BiasedSampleControl(x1,xDest,u);
  //BiasedSampleControl(xDest,x1,u);
}

//void KinodynamicCSpaceSentinelAdaptor::XDerivative(const State& x, const ControlInput& u, State& dx){
Matrix4 KinodynamicCSpaceSentinelAdaptor::SE3Derivative(const ControlInput& u)
{

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
  
  //#########################################################################
  Matrix4 dx_se3 = (X1*u(0) + X2*u(1) + X3*u(2) + X4*u(3) + X5*u(4) + X6*u(5));
  //#########################################################################

  return dx_se3;
}

//SE(3) element: X Y Z rotZ rotY rotX
//SE(3) element: X Y Z yaw pitch roll
Matrix4 KinodynamicCSpaceSentinelAdaptor::StateToSE3(const State& x){
  //represent lie group element as matrix4
  RigidTransform T_x_se3;
  Matrix3 R;

  EulerAngleRotation Reuler(x(3),x(4),x(5));
  Reuler.getMatrixZYX(R);

  T_x_se3.setTranslation(Vector3(x(0),x(1),x(2)));
  T_x_se3.setRotation(R);

  Matrix4 x_se3;
  T_x_se3.get(x_se3);
  return x_se3;
}
void KinodynamicCSpaceSentinelAdaptor::SE3ToState(State& x, const Matrix4& x_SE3){
  Matrix3 R_m;
  Vector3 T;
  x_SE3.get(R_m,T);

  EulerAngleRotation R;
  R.setMatrixZYX(R_m);

  x.resize(x.size());
  x.setZero();
  x(0)=T[0];
  x(1)=T[1];
  x(2)=T[2];

  x(3)=R[0];
  x(4)=R[1];
  x(5)=R[2];

  if(x(3)<-M_PI) x(3)+=2*M_PI;
  if(x(3)>M_PI) x(3)-=2*M_PI;

  if(x(4)<-M_PI/2) x(4)+=M_PI;
  if(x(4)>M_PI/2) x(4)-=M_PI;

  if(x(5)<-M_PI) x(5)+=2*M_PI;
  if(x(5)>M_PI) x(5)-=2*M_PI;

}
Real KinodynamicCSpaceSentinelAdaptor::Distance(const Config& x, const Config& y) { 
  //return base->Distance(x,y); 
 // Config xpos;xpos.resize(3);xpos(0)=x(0);xpos(1)=x(1);xpos(2)=x(2);
 // Config ypos;ypos.resize(3);ypos(0)=y(0);ypos(1)=y(1);ypos(2)=y(2);
 // return base->Distance(xpos,ypos); 

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
void KinodynamicCSpaceSentinelAdaptor::Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps){
  //dt = Rand(0.01,0.1);
  dt = 0.1;
  numSteps = 1;
}
///Randomly pick a control input
void KinodynamicCSpaceSentinelAdaptor::SampleControl(const State& x,ControlInput& u){
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
}
void KinodynamicCSpaceSentinelAdaptor::BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u){
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
}
