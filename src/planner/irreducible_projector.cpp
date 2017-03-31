#include "planner/irreducible_projector.h"
#include "elements/path_pwl_euclid.h"

IrreducibleProjector::IrreducibleProjector(Robot *robot){
  _robot = robot;
}
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
std::vector<Config> IrreducibleProjector::getSubLinkKeyframes(ForceFieldBackend &backend, std::vector<double> &lengths)
{
  uint N = _robot->links.size();
  Ndim = _robot->links.size();
  uint Nsub = N - _rootPath.at(0).size();
  uint Nhead = _rootPath.at(0).size();
  uint Nbranches = 1;
  uint Nsubdimension = Nsub/Nbranches;
  uint Nsegments= (Nsubdimension - 2)/3+1;
  assert(Nsub/Nbranches==(int)Nsub/Nbranches);

  std::vector<Vector3> rootPathEx;
  std::vector<Vector3> rootPathEy;
  std::vector<Vector3> rootPathEz;
  std::vector<Config> rootPathXYZ;
  std::vector<Matrix3> rootPathRot;

  vector<string> linkNames = _robot->linkNames;
  uint rootLinkId = 9;

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "IrreducibleProjector" << std::endl;
  std::cout << "ROOT LINK IS: " << linkNames[rootLinkId] << std::endl;
  std::cout << "obtaining " << _Nkeyframes << " keyframes" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  //Insert extended path to make starting positions irreducible
  double length_extended = 0.0;
  for(int i = 0; i < lengths.size(); i++){
    length_extended += lengths.at(i);
  }
  length_extended *= 2;

  Config q0 = _rootPath.at(0);
  Vector3 T0 = GetPositionAtLink(q0, rootLinkId);
  Matrix3 R0 = GetRotationAtLink(q0, rootLinkId);


  uint Nextended = 4;
  for(int j = Nextended; j > 0; j--){
    rootPathRot.push_back(R0);

    Vector3 ex = R0*Vector3(1,0,0);
    Vector3 Text = T0 - ex*(j*length_extended/Nextended);
    Config qp;qp.resize(3);
    qp(0) = Text[0];
    qp(1) = Text[1];
    qp(2) = Text[2];

    rootPathXYZ.push_back(qp);
    std::cout << (T0-Text).norm() << std::endl;
  }


  for(int i = 0; i < _rootPath.size(); i++){
    Config qr = _rootPath.at(i);

    Vector3 T0 = GetPositionAtLink(qr, rootLinkId);
    Matrix3 R0 = GetRotationAtLink(qr, rootLinkId);

    Config qpos;qpos.resize(3);
    qpos(0) = T0[0];
    qpos(1) = T0[1];
    qpos(2) = T0[2];

    rootPathXYZ.push_back(qpos);
    rootPathRot.push_back(R0);

  }
  //for(int i = 0; i < rootPathXYZ.size(); i++){
  //  Vector3 qpos;
  //  qpos[0] = rootPathXYZ.at(i)(0);
  //  qpos[1] = rootPathXYZ.at(i)(1);
  //  qpos[2] = rootPathXYZ.at(i)(2);
  //  Matrix3 R = rootPathRot.at(i);
  //  Vector3 ex = R*Vector3(1,0,0);
  //  Vector3 ey = R*Vector3(0,1,0);
  //  Vector3 ez = R*Vector3(0,0,1);
  //  backend.VisualizeFrame(qpos,ex,ey,ez);
  //}

  PathPiecewiseLinearEuclidean rpath = PathPiecewiseLinearEuclidean::from_keyframes(rootPathXYZ);

  std::cout << "Lengths : " << lengths << std::endl;
  std::cout << "Path has been extended by " << length_extended << "m backwards" << std::endl;

  std::vector<std::vector<double> > thetas;
  std::vector<std::vector<double> > gammas;
    //compute the projected theta/gamma values for specific point t0
  std::vector<double> lvec = rpath.GetLengthVector();
  std::cout << lvec.size() << std::endl;
  std::cout << Nextended << std::endl;
  double lall=0.0;
  for(int i = 0; i < Nextended-1; i++){
    lall+=lvec.at(i);
  }
  double t0 = lall;
  for(int i = Nextended-1; i < lvec.size(); i++)
  {
    t0 += lvec.at(i);
    uint idx = 0;

    //start at t0, move backwards to get previous link positions
    double dk = 0.0;
    Vector3 qlast = rpath.EvalVec3(t0-dk);

    Matrix3 R = rootPathRot.at(idx);
    std::vector<double> thetasAtT;
    std::vector<double> gammasAtT;
    for(int k = 0; k < lengths.size(); k++){

      Vector3 ex0 = R*Vector3(1,0,0);
      Vector3 ey0 = R*Vector3(0,1,0);
      Vector3 ez0 = R*Vector3(0,0,1);

      if(i==Nextended-1 || i==Nextended-1+20 || i==lvec.size()-1){
        Vector3 qk = rpath.EvalVec3(t0-dk);
        backend.VisualizeFrame(qk,ex0,ey0,ez0);
      }

      dk += lengths.at(k);
      Vector3 qk = rpath.EvalVec3(t0-dk);


      Vector3 dq = qk - qlast;
      Vector3 dqn = dq/dq.norm();

      double dx = dot(dqn, ex0);
      double dy = dot(dqn, ey0);
      double dz = dot(dqn, ez0);

      //get vectors projected into the xy and yz plane
      Vector3 qxy = dqn - dot(dqn,ez0)*dz;
      Vector3 qxz = dqn - dot(dqn,ey0)*dy;
      qxy /= qxy.norm();
      qxz /= qxz.norm();

      //compute angles
      double theta = acos( dot(qxy,-ex0));
      double gamma = acos( dot(qxz,-ex0));
      if(dot(dqn,ez0)<0) gamma = -gamma;
      if(dot(dqn,ey0)>0) theta = -theta;

      thetasAtT.push_back(theta);
      gammasAtT.push_back(gamma);

      Matrix3 Rz;Rz.setRotateZ(theta);
      Matrix3 Ry;Ry.setRotateY(gamma);
      R = Ry*Rz*R;

      qlast = qk;


    }
    gammas.push_back(gammasAtT);
    thetas.push_back(thetasAtT);
  }
//PYTHON CODE
//        pb = ft-f0
//        pbn = pb/np.linalg.norm(pb)
//        pa = np.dot(R.T,pbn)
//
//        xl = np.array((-1,0,0))
//        [theta,gamma] = getZYsphericalRot(pa, xl)
//                ap,xl
//                //pxy = ap-np.dot(ap,ze)*ze
                  //pxy = pxy/np.linalg.norm(pxy)
                  //pzx = ap-np.dot(ap,ye)*ye
                  //pzx = pzx/np.linalg.norm(pzx)
                  //xln = xl/np.linalg.norm(xl)

                  //theta = acos( np.dot(pxy,xln) )
                  //phi = acos( np.dot(pzx,xln) )
                  //if np.dot(ap,ze) < 0:
                  //        phi = -phi
                  //if np.dot(ap,ye) > 0:
                  //        theta = -theta
                  //return [theta,phi]

//
//        BC_R = np.dot(Ry(gamma),Rz(theta))
//
//        R = np.dot(R, BC_R)
//        ### compute the frame at t
//        dftnew = np.dot(R,np.array((1,0,0)))
//        ddftnew = np.dot(R,np.array((0,1,0)))
//        dddftnew = np.dot(R,np.array((0,0,1)))
//
//        ##check results
//        dd = np.linalg.norm(f0-ft)
//        epsilon = 0.01
//        if np.linalg.norm(dd-l) > epsilon:
//                print "distance between points on trajectory bigger than length between links"
//                print "distance between pts: ",dd
//                print "distance between links: ",l
//                print "epsilon precision:",epsilon
//                sys.exit(0)
//
//        return [theta,gamma,R]

  //theta,gammas to Config


  std::vector<Config> wholeBodyPath;
  for(int i = 0; i < _rootPath.size(); i++){
    Config q;
    q.resize(_robot->q.size());
    q.setZero();
    Config qhead = _rootPath.at(i);
    for(int j = 0; j < Nhead; j++){
      q(j) = qhead(j);
    }

    uint offset = 9;
    for(int j = 0; j < lengths.size(); j++){
      q(offset+3*j) = gammas.at(i).at(j);
      q(offset+3*j+1) = thetas.at(i).at(j);
      q(offset+3*j+2) = 0.0;
    }
    std::cout << q << std::endl;
    wholeBodyPath.push_back(q);
  }
  return wholeBodyPath;
}
