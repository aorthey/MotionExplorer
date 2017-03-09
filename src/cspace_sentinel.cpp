#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include "cspace_sentinel.h"

KinodynamicCSpaceSentinelAdaptor::KinodynamicCSpaceSentinelAdaptor(CSpace *_base): 
  KinematicCSpaceAdaptor(_base), type(Euler)
{
}

//////hooks for the math library integrators
////class MatrixDiffEqFunction
////{
////  public:
////      virtual ~MatrixDiffEqFunction() {}
////      virtual int NumDimensions() const { FatalError("NumDimensions not defined in subclass of DiffEqFunction"); return -1; }
////      virtual void operator ()(Real t,const Matrix4& y,Matrix4& fy) { PreEval(t,y); Eval(t,y,fy); }
////      virtual void PreEval(Real t,const Matrix4& y) {}
////      virtual void Eval(Real t,const Matrix4& y,Matrix4& fy) =0;
////};
////Matrix4 MatrixExponential(const Matrix4& x)
////{
////  Eigen::MatrixXd A(4,4);
////  for(int i = 0; i < 4; i++){
////    for(int j = 0; j < 4; j++){
////      A(i,j) = x(i,j);
////    }
////  }
////
////  Eigen::MatrixXd Aexp = A.exp();
////  Matrix4 result;
////  //std::cout << "The matrix exponential of A is:\n" << Aexp << "\n\n";
////  for(int i = 0; i < 4; i++){
////    for(int j = 0; j < 4; j++){
////      result(i,j) = Aexp(i,j);
////    }
////  }
////  return result;
////}
////
////
//////steps forward the initial value problem
//////y' = f(t,y)
//////y(t0) = w0
//////by the time step h, placing the result in w1
////void Euler_step(MatrixDiffEqFunction* f, Real t0, Real h, const Matrix4& w0, Matrix4& w1)
////{
////  Matrix4 k;
////  (*f)(t0,w0,k);
////  //w1=w0;
////  //w1.madd(k,h);
////  Matrix4 tmp = MatrixExponential(k*h);
////  w1 = w0*tmp;
////    //Matrix4 tmp = MatrixExponential(dw_se3*h);
////    //Matrix4 w1_SE3 = w0_SE3*tmp;
////    //State w1 = w0;
////    //SE3ToState(w1, w1_SE3);
////}
////
//////void RungeKutta4_step(MatrixDiffEqFunction* f, Real t0, Real h, const Matrix4& w0, Matrix4& w1)
//////{
//////  Assert(&w0 != &w1);
//////  Matrix4 k1,k2,k3,k4, tmp;
//////
//////  (*f)(t0,w0,k1);
//////
//////  //k1 *= h;
//////  //tmp = w0;
//////  //tmp.madd(k1,Half);
//////
//////  k1 *= h;
//////  //tmp = w0;
//////  mexp = MatrixExponential(k1*Half);
//////  tmp = w0*mexp;
//////
//////  (*f)(t0+h*Half, tmp, k2);
//////  k2 *= h;
//////  tmp = w0;
//////  tmp.madd(k2,Half);
//////
//////  k2 *= h;
//////  tmp = w0;
//////  tmp = MatrixExponential(k1*Half);
//////  tmp = w0*tmp;
//////
//////
//////
//////  (*f)(t0+h*Half, tmp, k3);
//////  k3 *= h;
//////
//////  tmp.add(w0,k3);
//////  (*f)(t0+h, tmp, k4);
//////  k4 *= h;
//////
//////  tmp.add(k2,k3);
//////  tmp.add(tmp,tmp);
//////  w1.add(k1,tmp);
//////  w1 += k4;
//////  w1.inplaceMul(1.0/6.0);
//////  w1 += w0;
//////}
////
////
////
////struct IntegrationFunction : public MatrixDiffEqFunction
////{
////  IntegrationFunction(KinodynamicCSpaceSentinelAdaptor* _space,const ControlInput& _u)
////    :space(_space),u(_u)
////  {}
////
////  virtual void Eval(Real t,const Matrix4& y,Matrix4& dy) {
////    dy = space->SE3Derivative(u);
////    //space->XDerivative(y,u,dy);
////  }
////
////  KinodynamicCSpaceSentinelAdaptor* space;
////  const ControlInput& u;
////};
////void KinodynamicCSpaceSentinelAdaptor::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
////{
////  //State x0 lies on local chart represented by R^6. We will convert x0
////  //to a R^4x4 matrix representing the same SE(3) element. Using the matrix
////  //representation allows easier computation of the forward dynamics. Once we
////  //are done, we reconvert the matrix representation to an element in vector
////  //representation in R^6 on the local chart.
////  //
////  // p is the path segment containing (numsteps) states along the forward simulation
////  // dt is the total timestep size
////  // numsteps is the number intermediate steps
////  // h = dt/numsteps is the intermediate timestep size
////
////  IntegrationFunction func(this,u);
////  std::cout << std::setprecision(2) << std::fixed;
////
////  Real dt;
////  int numSteps;
////  Parameters(x0,u,dt,numSteps);
////  Real h = dt/numSteps;
////
////  Matrix4 x0_SE3 = StateToSE3(x0);
////  std::vector<Matrix4> p_SE3;
////  p_SE3.push_back(x0_SE3);
////
////  for(int i=0;i<numSteps;i++) {
////    Matrix4 pnext;  
////    Matrix4 p0 = p_SE3.back();
////    Euler_step(&func,0,h,p0,pnext);
////    //RungeKutta4_step(&func,0,h,p0,pnext);
////
////    p_SE3.push_back(pnext);
////
////    //State w0 = p.back(); //\in SE(3) current element along path segment
////    //Matrix4 w0_SE3 = StateToSE3(w0);
////
////    //Matrix4 dw_se3 = this->SE3Derivative(u);
////    //Matrix4 tmp = MatrixExponential(dw_se3*h);
////    //Matrix4 w1_SE3 = w0_SE3*tmp;
////    //State w1 = w0;
////    //SE3ToState(w1, w1_SE3);
////    //p.push_back(w1);
////
////  }
////  for(int i = 0; i < p_SE3.size(); i++){
////    State pi;
////    SE3ToState(pi, p_SE3.at(i));
////    p.push_back(pi);
////  }
////
////  //exit(0);
////
////}

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


  for(int i = 0; i < p_SE3.size(); i++){
    State pi = x0;
    SE3ToState(pi, p_SE3.at(i));
    p.push_back(pi);
  }

  //exit(0);

}



/*
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

  Real dt = u(6);
  int numSteps = 1;
  //Parameters(x0,u,dt,numSteps);

  Real h = dt/numSteps;
  p.push_back(x0);

  std::cout << std::setprecision(2) << std::fixed;

  bool DEBUG=false;
  for(int i=0;i<numSteps;i++) {

    State w0 = p.back(); //\in SE(3) current element along path segment
    Matrix4 w0_SE3 = StateToSE3(w0);

    if(DEBUG) std::cout << std::string(80, '-') << std::endl;
    if(DEBUG) std::cout << w0 << std::endl;
    if(DEBUG) std::cout << w0_SE3 << std::endl;

    Matrix4 dw_se3 = this->SE3Derivative(u);
    Matrix4 tmp = MatrixExponential(dw_se3*h);
    Matrix4 w1_SE3 = w0_SE3*tmp;
    State w1 = w0;
    SE3ToState(w1, w1_SE3);
    p.push_back(w1);

    if(DEBUG) std::cout << std::string(10, '-') << std::endl;
    if(DEBUG) std::cout << w1 << std::endl;
    if(DEBUG) std::cout << w1_SE3 << std::endl;
  }

  //exit(0);

}
//*/



bool KinodynamicCSpaceSentinelAdaptor::ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p)
{
  throw("NYI");
//  Real dt;
//  int numSteps;
//  Parameters(x1,u,dt,numSteps);
//
//  Real h = -dt/numSteps;
//
//  p.push_back(x1);
//  for(int i=0;i<numSteps;i++) {
//
//    State w1 = p.back(); //\in SE(3) current element along path segment
//
//    Matrix4 w1_SE3 = StateToSE3(w1);
//    Matrix4 dw_se3 = this->SE3Derivative(u);
//    Matrix4 w0_SE3 = MatrixExponential(dw_se3*h)*w1_SE3;
//    State w0 = w1;
//    SE3ToState(w0, w0_SE3);
//    p.push_back(w0);
//    //std::cout << "STATE:" << w1 << std::endl;
//  }
//  //reverse the vector?
//  //std::reverse(p.begin(),p.end());
//  throw("NYI");
//
  return true;

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
  //return new GivenPathEdgePlanner(this,p,tolerance);
  return new TrueEdgePlanner(this,p.front(),p.back());
}

bool KinodynamicCSpaceSentinelAdaptor::IsValidControl(const State& x,const ControlInput& u){
  return true;
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
  if(!valid){
    std::cout << "Non valid control!" << std::endl;
    std::cout << u << std::endl;
    exit(0);
  }
  return valid;

}

//bool KinodynamicCSpaceSentinelAdaptor::ConnectionControl(const State& x,const State& xGoal,ControlInput& u)
//{
//  std::cout << "ERROR NYI" << std::endl;
//  exit(0);
//}


void KinodynamicCSpaceSentinelAdaptor::BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u){
  //SampleControl(x,u);
  //std::cout << xGoal << std::endl;

  //Node* goal = base->goal.root;
  int numSamples = 100;
  Real closest=Inf;

  //std::cout << "Going from "<< x << std::endl;
  //std::cout << "to "<< xGoal << std::endl;
  for(int i=0;i<numSamples;i++) {
    State x2;
    ControlInput temp;
    SampleControl(x,temp);
    SimulateEndpoint(x,temp,x2);
    Real dist = base->Distance(x2,xGoal);
    if(dist < closest) {
      closest = dist;
      u = temp;
    }
  }
  //std::cout << "best control "<< u << std::endl;
  //exit(0);
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

  // EulerAngleRotation Reuler(x(3),x(4),x(5));
  // Reuler.getMatrixZYX(R);
  EulerAngleRotation Reuler(x(5),x(4),x(3));
  Reuler.getMatrixXYZ(R);
  //R = Rz*Ry*Rx;
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
  R.setMatrixXYZ(R_m);

  x.resize(x.size());
  x.setZero();
  x(0)=T[0];
  x(1)=T[1];
  x(2)=T[2];

  x(3)=R[2];
  x(4)=R[1];
  x(5)=R[0];
}
Real KinodynamicCSpaceSentinelAdaptor::Distance(const Config& x, const Config& y) { 
  //return base->Distance(x,y); 
 // Config xpos;xpos.resize(3);xpos(0)=x(0);xpos(1)=x(1);xpos(2)=x(2);
 // Config ypos;ypos.resize(3);ypos(0)=y(0);ypos(1)=y(1);ypos(2)=y(2);
 // return base->Distance(xpos,ypos); 


  RigidTransform Ta,Tb;
  ConfigToTransform(x,Ta);
  ConfigToTransform(y,Tb);
  Real d = Ta.t.distance(Tb.t);
  Matrix3 Rrel;
  Rrel.mulTransposeB(Ta.R,Tb.R);
  AngleAxisRotation aa;
  aa.setMatrix(Rrel);
  double wt = 1;
  double wr = 0.01;
  d = Sqrt(d*d*wt + aa.angle*aa.angle*wr);
  //std::cout << " estimated : " << d << std::endl;
  return d;
}
///Randomly pick a control input
void KinodynamicCSpaceSentinelAdaptor::SampleControl(const State& x,ControlInput& u){
  double ak = 1;
  u.resize(x.size()+1);
  u.setZero();
  u(0) = 0;
  u(1) = Rand(-ak,+ak);
  u(2) = Rand(-ak,+ak);
  u(3) = 1;
  u(4) = 0;
  u(5) = 0;
  //T
  //u(6) = Rand(0.01,0.2);
  u(6) = Rand(0.2,0.8);
}
void KinodynamicCSpaceSentinelAdaptor::Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps){
  //dt = Rand(0.01,0.1);
  dt = 0.1;
  numSteps = 1;
}
