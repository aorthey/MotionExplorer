#include "planner/cspace/cspace_geometric_SE3.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SE2StateSpace.h>

GeometricCSpaceOMPLSE3::GeometricCSpaceOMPLSE3(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLSE3::initSpace()
{
  ob::StateSpacePtr SE3(std::make_shared<ob::SE3StateSpace>());
  this->space = SE3;
  ob::SE3StateSpace *cspaceSE3 = this->space->as<ob::SE3StateSpace>();

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> lowSE3;
  vector<double> highSE3;
  for(uint k = 0; k < 3; k++){
    lowSE3.push_back(minimum.at(k));
    highSE3.push_back(maximum.at(k));
  }

  ob::RealVectorBounds bounds(3);
  bounds.low = lowSE3;
  bounds.high = highSE3;
  cspaceSE3->setBounds(bounds);
  bounds.check();

}

ob::ScopedState<> GeometricCSpaceOMPLSE3::ConfigToOMPLState(const Config &q){

  ob::ScopedState<> qompl(space);
  ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::SE3StateSpace::StateType>();
  ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  qomplSE3->setXYZ(q(0),q(1),q(2));
  OMPLSO3StateSpaceFromEulerXYZ(q(3),q(4),q(5),qomplSO3);

  return qompl;
}

Config GeometricCSpaceOMPLSE3::OMPLStateToConfig(const ob::State *qompl){

  const ob::SE3StateSpace::StateType *qomplSE3 = qompl->as<ob::SE3StateSpace::StateType>();
  const ob::SO3StateSpace::StateType *qomplSO3 = &qomplSE3->rotation();

  Config q; q.resize(6);
  q(0) = qomplSE3->getX();
  q(1) = qomplSE3->getY();
  q(2) = qomplSE3->getZ();

  std::vector<double> rxyz = EulerXYZFromOMPLSO3StateSpace(qomplSO3);
  q(3) = rxyz.at(0);
  q(4) = rxyz.at(1);
  q(5) = rxyz.at(2);
  return q;
}
