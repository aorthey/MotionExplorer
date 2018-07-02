#include "irreducible_projector.h"

IrreducibleProjector::IrreducibleProjector(Robot *robot){
  _robot = robot;
}

pairDoubleVecVec IrreducibleProjector::ComputeThetaGammaFromRootPath( const std::vector<Vector3> &rootPos, const std::vector<Matrix3> &rootRot, const std::vector<double> &lengths)
{
  //###########################################################################
  //Compute Extension of Path
  //###########################################################################
  Vector3 T0 = rootPos.at(0);
  Matrix3 R0 = rootRot.at(0);

  //Insert extended path to make starting positions irreducible
  uint Nextended = 4;
  double length_extended = 0.0;
  for(uint i = 0; i < lengths.size(); i++){
    length_extended += lengths.at(i);
  }
  length_extended *= 2;
  for(uint j = 0; j < Nextended; j++){
    _rotationAlongRootPath.insert(_rotationAlongRootPath.begin(),R0);

    Vector3 ex = R0*Vector3(1,0,0);
    Vector3 Text = T0 - ex*(j*length_extended/Nextended);

    _positionAlongRootPath.insert(_positionAlongRootPath.begin(),Text);
  }

  //###########################################################################
  //piecewise linear interp between rootPos
  //###########################################################################
  std::cout << "Need to feed rootPos into PWL" << std::endl;
  exit(0);
  //PathPiecewiseLinear* rpath = PathPiecewiseLinear::from_keyframes(rootPos);

  //doubleVecVec thetas;
  //doubleVecVec gammas;

  ////compute the projected theta/gamma values for specific point t0
  //std::vector<double> lvec = rpath->GetLengthVector();

  //double lall=0.0;
  //for(uint i = 0; i < Nextended-1; i++){
  //  lall+=lvec.at(i);
  //}
  //double t0 = lall;
  //for(uint i = Nextended-1; i < lvec.size(); i++)
  //{
  //  //start at t0, move backwards to get previous link positions
  //  t0 += lvec.at(i);
  //  Matrix3 R = rootRot.at(i+1);

  //  pairDoubleVec thetagamma;
  //  thetagamma = ComputeThetaGammaFromRootPathPosition(*rpath, t0, R, lengths);

  //  thetas.push_back(thetagamma.first);
  //  gammas.push_back(thetagamma.second);
  //}
  //pairDoubleVecVec tg;
  //tg.first = thetas;
  //tg.second = gammas;
  //return tg;

}

pairDoubleVec IrreducibleProjector::ComputeThetaGammaFromRootPathPosition(const PathPiecewiseLinear &path, double t0, const Matrix3 &R0, const std::vector<double> &lengths)
{
  double dk = 0.0;
  Vector3 qlast = path.EvalVec3(t0-dk);

  doubleVec thetasAtT;
  doubleVec gammasAtT;

  Matrix3 R = R0;
  for(uint k = 0; k < lengths.size(); k++){

    Vector3 ex(1,0,0), ey(0,1,0), ez(0,0,1), ex0, ey0, ez0;
    R.mul(ex, ex0);
    R.mul(ey, ey0);
    R.mul(ez, ez0);

    dk += lengths.at(k);
    Vector3 qk = path.EvalVec3(t0-dk);

    Vector3 dq = qk - qlast;
    Vector3 dqn = dq/dq.norm();

    //get vectors projected into the xy and xz plane
    Vector3 qxy = dqn - dot(dqn,ez0)*ez0;
    Vector3 qxz = dqn - dot(dqn,ey0)*ey0;
    qxy /= qxy.norm();
    qxz /= qxz.norm();

    //compute angles
    double theta = acos( dot(qxy,-ex0));
    if(dot(dqn,ey0)>0) theta = -theta;
    Matrix3 Rz;Rz.setRotateZ(theta);

    double gamma = acos( dot(qxz,-ex0));
    if(dot(dqn,ez0)<0) gamma = -gamma;
    Matrix3 Ry;Ry.setRotateY(gamma);

    R = R*Ry*Rz;
    qlast = qk;

    thetasAtT.push_back(theta);
    gammasAtT.push_back(gamma);
  }
  pairDoubleVec tg;
  tg.first = thetasAtT;
  tg.second = gammasAtT;
  return tg;
}


//###########################################################################
// Klampt specifics
//###########################################################################
void IrreducibleProjector::setRootPath( std::vector<Config> &keyframes){
  _rootPath.clear();
  _rootPath = keyframes;
  _Nkeyframes = _rootPath.size();
}


Matrix3 IrreducibleProjector::GetRotationAtLink(const Config &q, uint id){
  uint Ndim = _robot->q.size();
  Config qreal;qreal.resize(Ndim);qreal.setZero();
  for(int j = 0; j < q.size(); j++) qreal(j)=q(j);
  _robot->UpdateConfig(qreal);
  vector<RobotLink3D> links = _robot->links;
  Frame3D qrootT = links[id].T_World;
  Matrix3 R = qrootT.R;
  return R;
}
Vector3 IrreducibleProjector::GetPositionAtLink(const Config &q, uint id){
  uint Ndim = _robot->q.size();
  Config qreal;qreal.resize(Ndim);qreal.setZero();
  for(int j = 0; j < q.size(); j++) qreal(j)=q(j);
  _robot->UpdateConfig(qreal);
  vector<RobotLink3D> links = _robot->links;
  Frame3D qrootT = links[id].T_World;
  Vector3 T = qrootT.t;
  return T;
}

