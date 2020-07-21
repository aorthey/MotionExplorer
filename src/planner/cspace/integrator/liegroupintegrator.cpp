#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include "planner/cspace/integrator/liegroupintegrator.h"

LieGroupIntegrator::LieGroupIntegrator()
{
  X1.setZero();
  X2.setZero();
  X3.setZero();
  X4.setZero();
  X5.setZero();
  X6.setZero();

  //XYZ Rz Ry Rx
  //##########################
  X1(0,3) = 1;
  //##########################
  X2(1,3) = 1;
  //##########################
  X3(2,3) = 1;
  //##########################
  X4(0,1) = -1;
  X4(1,0) = 1;
  //##########################
  X5(0,2) = 1;
  X5(2,0) = -1;
  //##########################
  X6(1,2) = -1;
  X6(2,1) = 1;
}

void LieGroupIntegrator::Euler_step(std::vector<Matrix4>& p, const Matrix4& dp0, double dt)
{  
  Matrix4 p0 = p.back();
  Matrix4 pnext = Integrate(p0,dp0,dt);
  p.push_back(pnext);
}

Matrix4 LieGroupIntegrator::Integrate(const Matrix4& p0, const Matrix4& dp0, double dt)
{
  Matrix4 tmp = MatrixExponential(dp0*dt);
  Matrix4 pnext = p0*tmp;
  return pnext;
}

void LieGroupIntegrator::Simulate(const State& x0, const ControlInput& u,std::vector<State>& p)
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
  Real h = dt/numSteps;

  Matrix4 x0_SE3 = StateToSE3(x0);
  Matrix4 dp0 = this->SE3Derivative(u);

  std::vector<Matrix4> p_SE3;
  p_SE3.push_back(x0_SE3);

  for(int i=0;i<numSteps;i++) {
    Euler_step(p_SE3, dp0, h);
  }

  p.push_back(x0);
  for(uint i = 1; i < p_SE3.size(); i++){
    State pi(x0);
    SE3ToState(pi, p_SE3.at(i));
    p.push_back(pi);
  }
}

Matrix4 LieGroupIntegrator::MatrixExponential(const Matrix4& x)
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

void LieGroupIntegrator::SimulateEndpoint(const State& x0, const ControlInput& u,State& x1)
{
  std::vector<State> p;
  Simulate(x0,u,p);
  x1 = p.back();
}

Matrix4 LieGroupIntegrator::SE3Derivative(const ControlInput& u)
{
  //#########################################################################
  Matrix4 dx_se3 = (X1*u(0) + X2*u(1) + X3*u(2) + X4*u(3) + X5*u(4) + X6*u(5));
  //#########################################################################

  return dx_se3;
}

//SE(3) element: X Y Z rotZ rotY rotX
//SE(3) element: X Y Z yaw pitch roll
Matrix4 LieGroupIntegrator::StateToSE3(const State& x){
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

void LieGroupIntegrator::SE3ToState(State& x, const Matrix4& x_SE3){
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
