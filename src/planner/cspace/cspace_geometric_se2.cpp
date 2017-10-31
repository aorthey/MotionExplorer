#include "planner/cspace/cspace_geometric_se2.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SE2StateSpace.h>

GeometricCSpaceOMPLSE2::GeometricCSpaceOMPLSE2(Robot *robot_, CSpace *space_):
  GeometricCSpaceOMPL(robot_, space_)
{
}

void GeometricCSpaceOMPLSE2::initSpace()
{
  //std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: SE(2)" << std::endl;

  ob::StateSpacePtr SE2 = (std::make_shared<ob::SE2StateSpace>());
  this->space = SE2;
  ob::SE2StateSpace *cspace = this->space->as<ob::SE2StateSpace>();

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> low;
  low.push_back(minimum.at(0));
  low.push_back(minimum.at(1));
  vector<double> high;
  high.push_back(maximum.at(0));
  high.push_back(maximum.at(1));

  ob::RealVectorBounds bounds(2);
  bounds.low = low;
  bounds.high = high;
  cspace->setBounds(bounds);
  bounds.check();

}
void GeometricCSpaceOMPLSE2::print()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "[CSpace]" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Name                      : OMPL CSPACE SE(2)" << std::endl;
  std::cout << "Dimensionality Space      :" << GetDimensionality() << std::endl;
}

ob::ScopedState<> GeometricCSpaceOMPLSE2::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  ob::SE2StateSpace::StateType *qomplSE2 = qompl->as<ob::SE2StateSpace::StateType>();

  qomplSE2->setXY(q(0),q(1));
  qomplSE2->setYaw(q(3));

  return qompl;
}

Config GeometricCSpaceOMPLSE2::OMPLStateToConfig(const ob::State *qompl){

  const ob::SE2StateSpace::StateType *qomplSE2 = qompl->as<ob::SE2StateSpace::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  q(0)=qomplSE2->getX();
  q(1)=qomplSE2->getY();
  q(3)=qomplSE2->getYaw();
  return q;

}
