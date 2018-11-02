#include "omplklamptconverter.h"
using namespace Math3D;

ob::State* ConfigToOMPLStatePtr(const Config &q, const ob::StateSpacePtr &s){
  ob::ScopedState<> qompl = ConfigToOMPLState(q, s);

  ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();
  ob::RealVectorStateSpace::StateType *qomplRn = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

  ob::State* out = s->allocState();
  ob::SE3StateSpace::StateType *outSE3 = out->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *outSO3 = &outSE3->rotation();
  ob::RealVectorStateSpace::StateType *outRn = out->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

  outSE3 = qomplSE3;
  outSO3 = qomplSO3;
  outRn = qomplRn;

  return out;
}
ob::ScopedState<> ConfigToOMPLState(const Config &q, const ob::StateSpacePtr &s){
  ob::ScopedState<> qompl(s);

  ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();
  ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  double* qomplRn = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRnSpace)->values;

  qomplSE3->setXYZ(q[0],q[1],q[2]);
  qomplSO3->setIdentity();

  //q SE3: X Y Z yaw pitch roll
  //double yaw = q[3];
  //double pitch = q[4];
  //double roll = q[5];

  Math3D::EulerAngleRotation Reuler(q(5),q(4),q(3));
  Matrix3 R;
  Reuler.getMatrixXYZ(R);

  QuaternionRotation qr;
  qr.setMatrix(R);

  double qx,qy,qz,qw;
  qr.get(qw,qx,qy,qz);

  qomplSO3->x = qx;
  qomplSO3->y = qy;
  qomplSO3->z = qz;
  qomplSO3->w = qw;

  //Math3D::Matrix3 qrM;
  //qr.getMatrix(qrM);

  for(int i = 0; i < q.size()-6; i++){
    qomplRn[i]=q(6+i);
  }
  return qompl;
}
Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::StateSpacePtr &s){
  const ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  //std::vector<double> reals;
  //s->copyToReals(reals, qomplRnState);
  uint N =  s->getDimension() - 6;
  Config q;
  q.resize(6+N);

  for(uint i = 0; i < N; i++){
    q(i+6) = qomplRnState->values[i];
  }

  q(0) = qomplSE3->getX();
  q(1) = qomplSE3->getY();
  q(2) = qomplSE3->getZ();

  double qx = qomplSO3->x;
  double qy = qomplSO3->y;
  double qz = qomplSO3->z;
  double qw = qomplSO3->w;

  Math3D::QuaternionRotation qr(qw, qx, qy, qz);
  Math3D::Matrix3 qrM;
  qr.getMatrix(qrM);
  Math3D::EulerAngleRotation R;
  R.setMatrixXYZ(qrM);

  q(3) = R[2];
  q(4) = R[1];
  q(5) = R[0];

  if(q(3)<-M_PI) q(3)+=2*M_PI;
  if(q(3)>M_PI) q(3)-=2*M_PI;

  if(q(4)<-M_PI/2) q(4)+=M_PI;
  if(q(4)>M_PI/2) q(4)-=M_PI;

  if(q(5)<-M_PI) q(5)+=2*M_PI;
  if(q(5)>M_PI) q(5)-=2*M_PI;

  return q;
}
Config OMPLStateToConfig(const ob::State *qompl, const ob::StateSpacePtr &s){
  const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  return OMPLStateToConfig(qomplSE3, qomplRnState, s);

}

Config OMPLStateToConfig(const ob::ScopedState<> &qompl, const ob::StateSpacePtr &s){

  const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
  const ob::RealVectorStateSpace::StateType *qomplRnState = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
  return OMPLStateToConfig(qomplSE3, qomplRnState, s);
}


void test()
{
 std::cout << "Testing Klampt->OMPL->Klampt conversion" << std::endl;

 Srand(0);
 for(int i = 0; i < 100; i++){
   uint N = Rand(0,0);
   ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
   ob::StateSpacePtr Rn(std::make_shared<ob::RealVectorStateSpace>(N));
   ob::StateSpacePtr stateSpace = SE3 + Rn;

   Config q;
   q.resize(6+N);
   q.setZero();
   q[0]=Rand(-3,3);
   q[1]=Rand(-3,3);
   q[2]=Rand(-3,3);
   q[3] = Rand(-M_PI,M_PI);
   q[4] = Rand(-M_PI/2,M_PI/2);
   q[5] = Rand(-M_PI,M_PI);
   for(uint j = 0; j < N; j++){
     q[j+6] = Rand(0,3);
   }
   //std::cout << "testing " << i << " : " << std::endl;
   test_conversion(q, stateSpace);
 }


}
void checkYawPitchRoll(double y, double p, double r, double y2, double p2, double r2)
{
  double epsilon = 1e-10;
  double dz = fabs(y-y2);
  double dy = fabs(p-p2);
  double dx = fabs(r-r2);
  //std::cout << "Original state: " << y << "," << p << "," << r << std::endl;
  //std::cout << "Convertd state: " << y2 << "," << p2 << "," << r2 << std::endl;

  if( (dz>epsilon) || (dy>epsilon) || (dx>epsilon))
  {
    std::cout << std::scientific;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Test failed" << std::endl;
    std::cout << dz << std::endl;
    std::cout << dy << std::endl;
    std::cout << dx << std::endl;
    std::cout << std::setprecision(5) << std::fixed;
    std::cout << "Original state: " << y << "," << p << "," << r << std::endl;
    std::cout << "Convertd state: " << y2 << "," << p2 << "," << r2 << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }
}
void testRotationConversion(){
  Srand(0);
  for(int i = 0; i < 100; i++){

    double roll = Rand(-M_PI,M_PI);

    double pitch = Rand(-M_PI/2,M_PI/2);

    double yaw = Rand(-M_PI,M_PI);

    Math3D::EulerAngleRotation Reuler(roll, pitch, yaw);
    Math3D::Matrix3 Rin;
    Reuler.getMatrixXYZ(Rin);

    double qx,qy,qz,qw;
    Math3D::QuaternionRotation qr;
    qr.setMatrix(Rin);
    qr.get(qw,qx,qy,qz);

    Math3D::QuaternionRotation qout(qw, qx, qy, qz);
    Math3D::Matrix3 qR;
    qout.getMatrix(qR);

    Math3D::EulerAngleRotation ReulerOut;
    ReulerOut.setMatrixXYZ(qR);

    Math3D::Matrix3 Rout;
    ReulerOut.getMatrixXYZ(Rout);

    double yaw2 = ReulerOut[2];
    double pitch2 = ReulerOut[1];
    double roll2 = ReulerOut[0];

    if(pitch2<-M_PI/2) pitch2+=M_PI;
    if(pitch2>M_PI/2) pitch2-=M_PI;
    if(roll2<-M_PI) roll2+=2*M_PI;
    if(roll2>M_PI) roll2-=2*M_PI;
    if(yaw2<-M_PI) yaw2+=2*M_PI;
    if(yaw2>M_PI) yaw2-=2*M_PI;

    double epsilon = 1e-10;
    if(!Rin.isEqual(Rout, epsilon)){
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Matrices differ" << std::endl;
      std::cout << Rin << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << Rout << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      checkYawPitchRoll(yaw,pitch,roll,yaw2,pitch2,roll2);
      exit(0);
    }

    checkYawPitchRoll(yaw,pitch,roll,yaw2,pitch2,roll2);
  
  }
}
void test_conversion(Config &q, ob::StateSpacePtr &stateSpace)
{
  ob::ScopedState<> qm = ConfigToOMPLState(q, stateSpace);
  Config qq = OMPLStateToConfig(qm, stateSpace);

  double epsilon = 1e-10;
  if((q-qq).norm()>epsilon){
    std::cout << "Klampt->OMPL->Klampt Test failed" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Klampt Input  state: " << q << std::endl;
    std::cout << "Klampt Output state: " << qq << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }
  ob::ScopedState<> qmout = ConfigToOMPLState(qq, stateSpace);
  if(qm != qmout){
    std::cout << "OMPL->Klampt->OMPL Test failed" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "OMPL Input  state: " << qm << std::endl;
    std::cout << "OMPL Output state: " << qmout << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    exit(0);
  }

}
