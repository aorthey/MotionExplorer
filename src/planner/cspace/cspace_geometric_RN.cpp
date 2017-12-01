#include "planner/cspace/cspace_geometric_RN.h"

#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLRN::GeometricCSpaceOMPLRN(RobotWorld *world_, int robot_idx, int dimension):
  GeometricCSpaceOMPL(world_, robot_idx), N(dimension)
{
}

void GeometricCSpaceOMPLRN::initSpace()
{
  //std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: SE(2)" << std::endl;

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

ob::ScopedState<> GeometricCSpaceOMPLRN::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  for(uint k = 0; k < N; k++){
    qomplRN->values[k] = q(k);
  }
  return qompl;
}

Config GeometricCSpaceOMPLRN::OMPLStateToConfig(const ob::State *qompl){

  const ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  for(uint k = 0; k < N; k++){
    q(k) = qomplRN->values[k];
  }
  return q;

}
