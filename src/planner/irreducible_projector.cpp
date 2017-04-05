#include "planner/irreducible_projector.h"

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
  for(int i = 0; i < lengths.size(); i++){
    length_extended += lengths.at(i);
  }
  length_extended *= 2;
  for(int j = 0; j < Nextended; j++){
    _rotationAlongRootPath.insert(_rotationAlongRootPath.begin(),R0);

    Vector3 ex = R0*Vector3(1,0,0);
    Vector3 Text = T0 - ex*(j*length_extended/Nextended);

    _positionAlongRootPath.insert(_positionAlongRootPath.begin(),Text);
  }

  //###########################################################################
  //piecewise linear interp between rootPos
  //###########################################################################
  PathPiecewiseLinearEuclidean rpath = PathPiecewiseLinearEuclidean::from_keyframes(rootPos);

  doubleVecVec thetas;
  doubleVecVec gammas;

  //compute the projected theta/gamma values for specific point t0
  std::vector<double> lvec = rpath.GetLengthVector();

  double lall=0.0;
  for(int i = 0; i < Nextended-1; i++){
    lall+=lvec.at(i);
  }
  double t0 = lall;
  for(int i = Nextended-1; i < lvec.size(); i++)
  {
    //start at t0, move backwards to get previous link positions
    t0 += lvec.at(i);
    Matrix3 R = rootRot.at(i+1);

    pairDoubleVec thetagamma;
    thetagamma = ComputeThetaGammaFromRootPathPosition(rpath, t0, R, lengths);

    thetas.push_back(thetagamma.first);
    gammas.push_back(thetagamma.second);
  }
  pairDoubleVecVec tg;
  tg.first = thetas;
  tg.second = gammas;
  return tg;

}

pairDoubleVec IrreducibleProjector::ComputeThetaGammaFromRootPathPosition(const PathPiecewiseLinearEuclidean &path, double t0, const Matrix3 &R0, const std::vector<double> &lengths)
{
  double dk = 0.0;
  Vector3 qlast = path.EvalVec3(t0-dk);

  doubleVec thetasAtT;
  doubleVec gammasAtT;

  Matrix3 R = R0;
  for(int k = 0; k < lengths.size(); k++){

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
  Config qreal;qreal.resize(Ndim);qreal.setZero();
  for(int j = 0; j < q.size(); j++) qreal(j)=q(j);
  _robot->UpdateConfig(qreal);
  vector<RobotLink3D> links = _robot->links;
  Frame3D qrootT = links[id].T_World;
  Matrix3 R = qrootT.R;
  return R;
}
Vector3 IrreducibleProjector::GetPositionAtLink(const Config &q, uint id){
  Config qreal;qreal.resize(Ndim);qreal.setZero();
  for(int j = 0; j < q.size(); j++) qreal(j)=q(j);
  _robot->UpdateConfig(qreal);
  vector<RobotLink3D> links = _robot->links;
  Frame3D qrootT = links[id].T_World;
  Vector3 T = qrootT.t;
  return T;
}
std::vector<Config> IrreducibleProjector::getSubLinkKeyframes(std::vector<double> &lengths, uint Nbranches)
{

  Ndim = _robot->links.size();

  uint Nsegments = lengths.size();
  uint NbranchOffset = 3*(Nsegments)+2;
//###########################################################################
// set zero config wholebodypath
//###########################################################################
  std::vector<Config> wholeBodyPath;
  for(int i = 0; i < _rootPath.size(); i++){
    Config q;
    q.resize(_robot->q.size());
    q.setZero();
    Config qhead = _rootPath.at(i);
    for(int j = 0; j < _rootPath.at(i).size(); j++){
      q(j) = qhead(j);
    }
    wholeBodyPath.push_back(q);
  }

//###########################################################################
// compute 
//###########################################################################
  for(int k = 0; k < Nbranches; k++){

    vector<string> linkNames = _robot->linkNames;
    uint rootLinkId = 9+k*NbranchOffset;

    std::cout << linkNames << std::endl;
    std::cout << NbranchOffset << std::endl;

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "IrreducibleProjector" << std::endl;
    std::cout << "obtaining " << _Nkeyframes << " keyframes" << std::endl << std::endl;

    std::cout << "Computing irreducible path from root link: " << linkNames[rootLinkId] << std::endl;
    std::cout << " root link has " << lengths.size() << " sublinks with lengths " << lengths << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    _positionAlongRootPath.clear();
    _rotationAlongRootPath.clear();

    for(int i = 0; i < _rootPath.size(); i++){
      Config qr = _rootPath.at(i);

      Vector3 T0 = GetPositionAtLink(qr, rootLinkId);
      Matrix3 R0 = GetRotationAtLink(qr, rootLinkId);

      _positionAlongRootPath.push_back(T0);
      _rotationAlongRootPath.push_back(R0);

    }

    pairDoubleVecVec thetagamma = ComputeThetaGammaFromRootPath( _positionAlongRootPath ,_rotationAlongRootPath, lengths);

    doubleVecVec thetas = thetagamma.first;
    doubleVecVec gammas = thetagamma.second;

    assert(thetas.size()==_rootPath.size());
    assert(gammas.size()==_rootPath.size());

    for(int i = 0; i < wholeBodyPath.size(); i++){

      for(int j = 0; j < lengths.size(); j++){
        double tij = thetas.at(i).at(j);
        double gij = gammas.at(i).at(j);
        wholeBodyPath.at(i)(rootLinkId+3*j) = tij;
        wholeBodyPath.at(i)(rootLinkId+3*j+1) = gij;
        wholeBodyPath.at(i)(rootLinkId+3*j+2) = 0.0;
      }
      //std::cout << wholeBodyPath.at(i) << std::endl;
    }


  }
  return wholeBodyPath;
}

