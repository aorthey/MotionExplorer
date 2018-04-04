#pragma once
#include <gtest/gtest.h>
#include <Eigen/Core>

void CompareEigenVector( Eigen::VectorXd x, Eigen::VectorXd y, double EPSILON_EQ = 1e-5)
{
  ASSERT_EQ(x.size(), y.size()) << "Vectors x and y are of unequal length";
  for (int i = 0; i < x.size(); ++i) {
    ASSERT_TRUE(fabs(x[i] - y[i]) <= EPSILON_EQ) << "Vectors x and y differ at index " << i;
  }
}
void CompareEigenMatrix( Eigen::MatrixXd x, Eigen::MatrixXd y, double EPSILON_EQ = 1e-5)
{
  ASSERT_EQ(x.rows(), y.rows()) << "Number of rows of x and y are of unequal length";
  ASSERT_EQ(x.cols(), y.cols()) << "Number of cols of x and y are of unequal length";
  for (int i = 0; i < x.rows(); ++i) {
    for (int j = 0; j < x.cols(); ++j) {
      ASSERT_TRUE(fabs(x(i,j) - y(i,j)) <= EPSILON_EQ) << "Matrices x and y differ at index " << i << "," << j;
    }
  }
}

