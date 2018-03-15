#include "planner/cspace/cspace_geometric_RN.h"

#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLRN::GeometricCSpaceOMPLRN(RobotWorld *world_, int robot_idx, int dimension):
  GeometricCSpaceOMPL(world_, robot_idx), N(dimension)
{
}

void GeometricCSpaceOMPLRN::initSpace()
{
  ob::StateSpacePtr RN = (std::make_shared<ob::RealVectorStateSpace>(N));
  this->space = RN;
  ob::RealVectorStateSpace *cspace = this->space->as<ob::RealVectorStateSpace>();

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> low;
  vector<double> high;
  for(uint k = 0; k < N; k++){
    low.push_back(minimum.at(k));
    high.push_back(maximum.at(k));
  }

  ob::RealVectorBounds bounds(N);
  bounds.low = low;
  bounds.high = high;
  cspace->setBounds(bounds);
  bounds.check();

}

void GeometricCSpaceOMPLRN::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  for(uint k = 0; k < N; k++){
    qomplRN->values[k] = q(k);
  }
}

Config GeometricCSpaceOMPLRN::OMPLStateToConfig(const ob::State *qompl){

  const ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  for(uint k = 0; k < N; k++){
    q(k) = qomplRN->values[k];
  }
  return q;

}
void GeometricCSpaceOMPLRN::print() const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            : R^" << GetDimensionality() << std::endl;

  ob::RealVectorStateSpace *cspace = space->as<ob::RealVectorStateSpace>();

  const ob::RealVectorBounds bounds = cspace->getBounds();
  std::vector<double> min = bounds.low;
  std::vector<double> max = bounds.high;
  std::cout << "RN bounds min     : ";
  for(uint i = 0; i < min.size(); i++){
    std::cout << " " << min.at(i);
  }
  std::cout << std::endl;

  std::cout << "RN bounds max     : ";
  for(uint i = 0; i < max.size(); i++){
    std::cout << " " << max.at(i);
  }
  std::cout << std::endl;

  std::cout << std::string(80, '-') << std::endl;
}
