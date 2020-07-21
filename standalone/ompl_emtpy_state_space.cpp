#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>


namespace ob = ompl::base;


int main(int argc,const char** argv)
{
  auto spaceEmpty(std::make_shared<ob::RealVectorStateSpace>(0));
  std::cout << spaceEmpty->getDimension() << std::endl;

  return 0;
}
