#include "planner/cspace/cspace_geometric_fixedbase.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLFixedBase::GeometricCSpaceOMPLFixedBase(RobotWorld *world_, int robot_idx, uint N_):
  GeometricCSpaceOMPL(world_, robot_idx)
{
  if(N_==0){
    N = robot->q.size()-6;
  }else{
    N = N_;
  }
}

void GeometricCSpaceOMPLFixedBase::initSpace()
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
    low.push_back(minimum.at(k+6));
    high.push_back(maximum.at(k+6));
  }

  ob::RealVectorBounds bounds(N);
  bounds.low = low;
  bounds.high = high;
  cspace->setBounds(bounds);
  bounds.check();

}

ob::ScopedState<> GeometricCSpaceOMPLFixedBase::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  for(uint k = 0; k < N; k++){
    qomplRN->values[k] = q(k+6);
  }
  return qompl;
}

Config GeometricCSpaceOMPLFixedBase::OMPLStateToConfig(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  Config q;q.resize(robot->q.size());q.setZero();
  for(uint k = 0; k < 6; k++){
    q(k) = 0;
  }
  for(uint k = 0; k < N; k++){
    q(k+6) = qomplRN->values[k];
  }
  return q;

}
