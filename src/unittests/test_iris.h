#pragma once
#include <Eigen/Core>
#include <iris/iris.h>
#include "test_util.h"

TEST(IRISTest, BoxTest) {
  iris::IRISProblem problem(3);
  // Inflate a region inside a 1x1x1 box
  problem.setSeedPoint(Eigen::Vector3d(0.1, 0.1, 0.1));

  //obstacle is defined by Matrix NxM whereby N is the number of dimensions, and
  //M are the number of points. It is assumed that the obstacle is convex, i.e.
  //the obstacle is defined as the convex hull of all the given points
  Eigen::MatrixXd obs(3,4);
  //left
  obs << 0, 0, 0, 0,
         0, 1, 1, 0,
         0, 1, 0, 1;
  problem.addObstacle(obs);
  //right
  obs << 1, 1, 1, 1,
         0, 1, 1, 0,
         0, 1, 0, 1;
  problem.addObstacle(obs);

  //front
  obs << 0, 1, 1, 0,
         0, 0, 0, 0,
         0, 0, 1, 1;
  problem.addObstacle(obs);
  //back
  obs << 0, 1, 1, 0,
         1, 1, 1, 1,
         0, 0, 1, 1;
  problem.addObstacle(obs);

  //bottom
  obs << 0, 1, 0, 1,
         0, 0, 1, 1,
         0, 0, 0, 0;
  problem.addObstacle(obs);
  //top
  obs << 0, 1, 0, 1,
         0, 0, 1, 1,
         1, 1, 1, 1;
  problem.addObstacle(obs);

  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);

  Eigen::MatrixXd A_eigen = region.polyhedron.getA();
  Eigen::VectorXd b_eigen = region.polyhedron.getB();
  Eigen::MatrixXd C_eigen = region.ellipsoid.getC();
  Eigen::VectorXd d_eigen = region.ellipsoid.getD();
  // std::cout << "A: " << A_eigen << std::endl;
  // std::cout << "b: " << b_eigen << std::endl;
  // std::cout << "C: " << C_eigen << std::endl;
  // std::cout << "d: " << d_eigen << std::endl;
  
  Eigen::VectorXd b(6);
  b << 0,0,0,1,1,1;
  Eigen::VectorXd d(3);
  d << 0.5,0.5,0.5;
  Eigen::MatrixXd C(3,3);
  C << 0.5, 0  , 0  ,
        0 , 0.5, 0  ,
        0 , 0  , 0.5;
  Eigen::MatrixXd A(6,3);
  A << -1 ,  0, 0 ,
        0 , -1, 0 ,
        0 , 0 , -1,
        0 , 1 , 0 ,
        1 , 0 , 0 ,
        0 , 0 , 1 ,
  // std::cout << "A: " << A << std::endl;
  // std::cout << "b: " << b << std::endl;
  // std::cout << "C: " << C << std::endl;
  // std::cout << "d: " << d << std::endl;

  //CompareEigenMatrix( A, A_eigen, 1e-2);
  CompareEigenVector( b, b_eigen, 1e-2);
  CompareEigenMatrix( C, C_eigen, 1e-2);
  CompareEigenVector( d, d_eigen, 1e-2);
}
