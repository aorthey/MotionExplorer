#include "planner/cspace/cspace_geometric_r2.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLR2::GeometricCSpaceOMPLR2(Robot *robot_, CSpace *space_):
  GeometricCSpaceOMPL(robot_, space_)
{
}

void GeometricCSpaceOMPLR2::initSpace()
{
  //std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: SE(2)" << std::endl;

  ob::StateSpacePtr R2 = (std::make_shared<ob::RealVectorStateSpace>(2));
  this->space = R2;
  ob::RealVectorStateSpace *cspace = this->space->as<ob::RealVectorStateSpace>();

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
void GeometricCSpaceOMPLR2::print()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "[CSpace]" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Name                      : OMPL CSPACE R^2" << std::endl;
  std::cout << "Dimensionality Space      :" << GetDimensionality() << std::endl;
}

ob::ScopedState<> GeometricCSpaceOMPLR2::ConfigToOMPLState(const Config &q){
  ob::ScopedState<> qompl(space);
  ob::RealVectorStateSpace::StateType *qomplR2 = qompl->as<ob::RealVectorStateSpace::StateType>();
  qomplR2->values[0] = q(0);
  qomplR2->values[1] = q(1);
  return qompl;
}

Config GeometricCSpaceOMPLR2::OMPLStateToConfig(const ob::State *qompl){

  const ob::RealVectorStateSpace::StateType *qomplR2 = qompl->as<ob::RealVectorStateSpace::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  q(0)=qomplR2->values[0];
  q(1)=qomplR2->values[1];
  return q;

}
