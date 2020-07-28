
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <unsupported/Eigen/Splines>
#include <Eigen/Geometry> 

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
    {
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);
    }

    matrix.conservativeResize(numRows,numCols);
}
void QuaternionToDirectionVector(
  const Eigen::Quaternion<double> q,
  Eigen::Vector3d &v)
{
  Eigen::Vector3d ex(1,0,0);
  v = q.toRotationMatrix()*ex;
}
void DirectionVectorToQuaternion(
  const Eigen::Vector3d v,
  Eigen::Quaternion<double> &q)
{
  Eigen::Vector3d ex(1,0,0);
  q = Eigen::Quaternion<double>::FromTwoVectors(ex, v);
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateQuasiSectionSpline(
      const base::State* xFiberStart,
      const base::State* xFiberGoal,
      const std::vector<base::State*> basePath) 
{
    
  if(bundleSpaceGraph_->getBase()->getStateSpace()->getType() != base::STATE_SPACE_REAL_VECTOR
      || (bundleSpaceGraph_->getBaseDimension() > 3))
  {
      throw Exception("NYI");
  }

  const int dim = bundleSpaceGraph_->getBaseDimension();

  // T1 exp(t log(T1^-1 T2))

  //############################################################################
  //BezierCurveFit through BasePath
  //############################################################################

  double spacingBetweenPoints = 1.0/double(basePath.size()-1);
  Eigen::RowVectorXd time = Eigen::RowVectorXd::Constant(basePath.size(), spacingBetweenPoints);

  time(0) = 0;

  for(uint k = 1; k < basePath.size(); k++)
  {
      base::State* bk = basePath.at(k);
      base::State* bkk = basePath.at(k-1);
      double dk = bundleSpaceGraph_->getBase()->distance(bk, bkk);
      time(k) = dk + time(k-1);
  }

  Eigen::MatrixXd states = Eigen::MatrixXd::Zero(dim, basePath.size());
  for(uint j = 0; j < basePath.size(); j++)
  {
      base::State* bj = basePath.at(j);
      double*& v = bj->as<base::RealVectorStateSpace::StateType>()->values;

      for(int k = 0; k < dim; k++)
      {
          states(k,j) = v[k];
      }
  }
  for(uint k = 1; k < basePath.size(); k++)
  {
    double dt = time(k)-time(k-1);
    if(dt < 1e-10)
    {
      removeColumn(states, k);
    }
  }

  //############################################################################
  //Extract Derivatives
  //############################################################################
  const base::SO3StateSpace::StateType *xFiberStart_SO3 =
       xFiberStart->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
  const base::SO3StateSpace::StateType *xFiberGoal_SO3 =
       xFiberGoal->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);

  Eigen::Quaternion<double> qStart(
      xFiberStart_SO3->w, 
      xFiberStart_SO3->x,
      xFiberStart_SO3->y,
      xFiberStart_SO3->z);
  Eigen::Quaternion<double> qGoal(
      xFiberGoal_SO3->w, 
      xFiberGoal_SO3->x,
      xFiberGoal_SO3->y,
      xFiberGoal_SO3->z);

  Eigen::Vector3d vStart, vGoal;
  QuaternionToDirectionVector(qStart, vStart);
  QuaternionToDirectionVector(qGoal, vGoal);

  //############################################################################
  //Eval Spline
  //############################################################################
  Eigen::MatrixXd derivatives = Eigen::MatrixXd::Constant(dim, basePath.size(), 1.0);

  derivatives.col(0).tail(3) = vStart;
  derivatives.col(basePath.size()-1).tail(3) = vGoal;

  for(uint k = 1; k < basePath.size()-1; k++)
  {
      Eigen::Vector3d pk;

      //Note: Catmull-Rom: use pk+1 - pk-1 / dist(k-1,k+1)
      Eigen::Vector3d pk_tangent = states.col(k+1) - states.col(k-1);
      pk = pk_tangent/(time(k+1) - time(k-1));//pk_tangent.squaredNorm();

      pk = pk/pk.norm();

      //
      //Note: Finite difference
      // Eigen::Vector3d pk_in = states.col(k) - states.col(k-1);
      // Eigen::Vector3d pk_out = states.col(k+1) - states.col(k);
      // double dk_in = pk_in.squaredNorm();
      // double dk_out = pk_out.squaredNorm();

      // if(dk_out < 1e-10){
      //   pk = derivatives.col(k-1).tail(3);
      // }else{
      //   pk = 0.5*(pk_in/dk_in + pk_out/dk_out);
      // }

      derivatives.col(k).tail(3) = pk;
  }

  // std::cout << std::string(80, '-') << std::endl;
  // std::cout << states << std::endl;
  // std::cout << std::string(80, '-') << std::endl;
  // std::cout << derivatives << std::endl;
  // std::cout << std::string(80, '-') << std::endl;


  // const SplineNd spline = 
  //   SplineFittingNd::Interpolate(states, 3);

  //############################################################################
  //Cubic Hermite Spline
  //############################################################################
  std::vector<base::State*> bundlePath;
  int N = 10;
  bundlePath.resize(N*(states.cols()-1));
  bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

  double x_last = 0;
  for(uint k = 0; k < states.cols()-1; k++)
  {
      Eigen::Vector3d p0 = states.col(k);
      Eigen::Vector3d p1 = states.col(k+1);
      Eigen::Vector3d m0 = derivatives.col(k);
      Eigen::Vector3d m1 = derivatives.col(k+1);

      double x_next = time(k+1);
      for(int j = 0; j < N; j++)
      {
          double x = x_last + (double(j)/double(N-1)) * (x_next - x_last);
          double t = (x - x_last) / (x_next - x_last);

          double tt = t*t;
          double ttt = t*t*t;

          double h00 = 2*ttt - 3*tt + 1;
          double h10 = ttt - 2*tt + t;
          double h01 = 3*tt - 2*ttt;
          double h11 = ttt - tt;

          double dx = x_next - x_last;

          Eigen::Vector3d pt = h00*p0 + h10*dx*m0 + h01*p1 + h11*dx*m1;

          if(pt != pt)
          {
            std::cout << "NAN" << std::endl;
            exit(0);
          }

          double dh00 = 6*tt - 6*t;
          double dh10 = 3*tt - 4*t + 1;
          double dh01 = 6*t - 6*tt;
          double dh11 = 3*tt - 2*t;

          Eigen::Vector3d dpt = dh00*p0 + dh10*dx*m0 + dh01*p1 + dh11*dx*m1;

          if(dpt != dpt)
          {
            std::cout << "NAN" << std::endl;
            exit(0);
          }

          base::State* bj = bundlePath.at(j + k*N);
          base::SE3StateSpace::StateType *xBundle_SE3 =
             bj->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);

          xBundle_SE3->setXYZ(pt(0), pt(1), pt(2));

          base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();

          Eigen::Quaternion<double> qj;
          DirectionVectorToQuaternion(dpt, qj);

          xBundle_SO3->x = qj.x();
          xBundle_SO3->y = qj.y();
          xBundle_SO3->z = qj.z();
          xBundle_SO3->w = qj.w();

          base::RealVectorStateSpace::StateType *xBundle_RN =
             bj->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

          int K = bundleSpaceGraph_->getBundleDimension() - 6;
          for (int k = 0; k < K; k++)
          {
              xBundle_RN->values[k] = 0;
          }
          xBundle_RN->values[0] = 1;
      }
      x_last = x_next;
  }

  ////############################################################################
  ////Convert to OMPL
  ////############################################################################
  //const SplineNd spline = 
  //  SplineFittingNd::InterpolateWithDerivatives(states, derivatives, indices, splineDegree);

  //int N = 100;
  //std::vector<base::State*> bundlePath;
  //bundlePath.resize(N);
  //bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

  //double d = 0;
  //double dstep = 1.0/double(N-1);
  //for(int j = 0; j < N; j++)
  //{
  //    PointType pt = spline(d);

  //    base::State* bj = bundlePath.at(j);
  //    base::SE3StateSpace::StateType *xBundle_SE3 =
  //       bj->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);

  //    Eigen::Vector3d xj = pt.tail(3);
  //    xBundle_SE3->setXYZ(xj(0), xj(1), xj(2));

  //    base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();

  //    // PointType deriv = spline.derivatives(d, 1); //TODO: seems to not work
  //    // correctly
  //    Eigen::Vector3d dx;
  //    dx = ((spline(d + dstep) - spline(d))/dstep).tail(3);
  //    dx = dx.normalized();

  //    Eigen::Quaternion<double> qj;
  //    DirectionVectorToQuaternion(dx, qj);

  //    xBundle_SO3->x = qj.x();
  //    xBundle_SO3->y = qj.y();
  //    xBundle_SO3->z = qj.z();
  //    xBundle_SO3->w = qj.w();

  //    base::RealVectorStateSpace::StateType *xBundle_RN =
  //       bj->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

  //    int K = bundleSpaceGraph_->getBundleDimension() - 6;
  //    for (int k = 0; k < K; k++)
  //    {
  //        xBundle_RN->values[k] = 0;
  //    }
  //    xBundle_RN->values[0] = 1;

  //    d += dstep;
  //}
  return bundlePath;

}

