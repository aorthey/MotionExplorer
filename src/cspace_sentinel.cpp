#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include "cspace_sentinel.h"

KinodynamicCSpaceSentinelAdaptor::KinodynamicCSpaceSentinelAdaptor(CSpace *_base): 
  KinematicCSpaceAdaptor(_base), type(Euler)
{
}

void KinodynamicCSpaceSentinelAdaptor::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
{
  //State x0 lies on local chart and is represented by R^6. We will convert x0
  //to a R^4x4 matrix representing the same SE(3) element. Using the matrix
  //representation allows easier computation of the forward dynamics. Once we
  //are done, we reconvert the matrix representation to an element in vector
  //representation in R^6 on the local chart.
  //
  // p is the path segment containing (numsteps) states along the forward simulation
  // dt is the total timestep size
  // numsteps is the number intermediate steps
  // h = dt/numsteps is the intermediate timestep size

  Real dt;
  int numSteps;
  Parameters(x0,u,dt,numSteps);

  Real h = dt/numSteps;
  p.push_back(x0);

  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "START STATE:"<<x0 << std::endl;
  for(int i=0;i<numSteps;i++) {

    State w0 = p.back(); //\in SE(3) current element along path segment

    Matrix4 w0_SE3 = StateToSE3(w0);
    Matrix4 dw_se3 = this->SE3Derivative(w0_SE3,u);
    //Matrix4 w1_SE3 = w0_SE3 + MatrixExponential(dw_se3*h);
    Matrix4 w1_SE3 = MatrixExponential(dw_se3*h*i)*w0_SE3;
    State w1 = w0;
    SE3ToState(w1, w1_SE3);
    std::cout << "STATE:" << w1 << std::endl;
    //Matrix4 w1_SE3 = 
    //State w1=w0; //\in SE(3) next element
    //w1.madd(dw,h); //w1 += dw*h
    p.push_back(w1);
  }
  //exit(0);

}
Matrix4& KinodynamicCSpaceSentinelAdaptor::MatrixExponential(const Matrix4& x)
{
  Eigen::MatrixXd A(4,4);
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      A(i,j) = x(i,j);
    }
  }

  Eigen::MatrixXd Aexp = A.exp();
  Matrix4 result;
  //std::cout << "The matrix exponential of A is:\n" << A.exp() << "\n\n";
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
  double ak = 0.1;
  u.resize(x.size());
  u.setZero();
  u(0) = 0;
  u(1) = 0;//Rand(-ak,ak);
  u(2) = Rand(-ak,ak);
  u(3) = 1;
  u(4) = 0;
  u(5) = 0;
}

void KinodynamicCSpaceSentinelAdaptor::BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u){
  SampleControl(x,u);
}
void KinodynamicCSpaceSentinelAdaptor::BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u){
  BiasedSampleControl(x1,xDest,u);
}

//void KinodynamicCSpaceSentinelAdaptor::XDerivative(const State& x, const ControlInput& u, State& dx){
Matrix4 KinodynamicCSpaceSentinelAdaptor::SE3Derivative(const Matrix4& x_SE3, const ControlInput& u)
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
  

  bool DEBUG=false;

  //#########################################################################
  //Matrix4 dx_se3 = x_SE3*(X1*u(0) + X2*u(1) + X3*u(2) + X4);
  //Matrix4 dx_se3 = x_SE3*(X3*u(2) + X4);
  //Matrix4 dx_se3 = x_SE3*(X4);
  Matrix4 dx_se3 = (X1*u(0) + X2*u(1) + X3*u(2) + X4);
  //#########################################################################

  return dx_se3;
  ////dx_se3 is a lie algebra element represented as matrix4
  //// need to convert back to Vector6 element
  //Matrix3 dR_m;
  //EulerAngleRotation dR;
  //Vector3 dT;
  //dx_se3.get(dR_m,dT);

  //dR.setMatrixXYZ(dR_m);

  //dx.resize(x.size());
  //dx.setZero();

  //dx(0)=dT[0];
  //dx(1)=dT[1];
  //dx(2)=dT[2];
  //dx(3)=dR[0];
  //dx(4)=dR[1];
  //dx(5)=dR[2];

  //if(DEBUG){
  ////if(x(3)>0.5){
  //  std::cout << "X1:" << std::endl <<  X1*u(0) << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X2:" << std::endl <<  X2*u(1) << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X3:" << std::endl <<  X3*u(2) << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X4:" << std::endl <<  X4*u(3) << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X5:" << std::endl <<  X5 << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X6:" << std::endl <<  X6 << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << std::setprecision(2) << std::fixed;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X input:" << std::endl << x << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X SE(3) input:" << std::endl << x_se3 << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "U input:" << std::endl << u << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  //Matrix4 u_se3 = X1*u(0) + X2*u(1) + X3*u(2) + X4;
  //  //std::cout << "U SE(3) input:" << std::endl << u_se3 << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "DX se(3) output:" << std::endl << dx_se3 << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "DX output:" << dx << std::endl;
  //  std::cout << std::string(80, '-') << std::endl;
  //  std::cout << "X1 output:" << x+0.01*dx << std::endl;
  ////std::cout << std::string(80, '-') << std::endl;
  //  //
  //  exit(0);
  //}

}
Matrix4 KinodynamicCSpaceSentinelAdaptor::StateToSE3(const State& x){
  //represent lie group element as matrix4
  RigidTransform T_x_se3;
  Matrix3 R,Rx,Ry,Rz;
  Rx.setRotateX(x(3));
  Ry.setRotateY(x(4));
  Rz.setRotateZ(x(5));
  R = Rz*Ry*Rx;
  T_x_se3.setRotation(R);
  T_x_se3.setTranslation(Vector3(x(0),x(1),x(2)));

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
  x(3)=R[0];
  x(4)=R[1];
  x(5)=R[2];

}
void KinodynamicCSpaceSentinelAdaptor::Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps){
  dt = 0.001;
  numSteps = 10;
}
