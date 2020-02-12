#include "planner/cspace/cspace_geometric_R_CONTACT.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "ompl/base/spaces/SE2StateSpaceFullInterpolate.h"

GeometricCSpaceOMPLRCONTACT::GeometricCSpaceOMPLRCONTACT(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLRCONTACT::initSpace()
{
  std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: R CONTACT" << std::endl;

  ompl::base::StateSpacePtr r5(new ompl::base::RealVectorStateSpace(5));
  ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
  ob::StateSpacePtr R_CONTACT = r5 + so2;
  this->space = R_CONTACT;
  ob::SE2StateSpaceFullInterpolate *cspace = this->space->as<ob::SE2StateSpaceFullInterpolate>();

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

void GeometricCSpaceOMPLRCONTACT::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SE2StateSpaceFullInterpolate::StateType *qomplSE2 = qompl->as<ob::SE2StateSpaceFullInterpolate::StateType>();

  qomplSE2->setXY(q(0),q(1));
  qomplSE2->setYaw(q(3));

}

Config GeometricCSpaceOMPLRCONTACT::OMPLStateToConfig(const ob::State *qompl){

  const ob::SE2StateSpaceFullInterpolate::StateType *qomplSE2 = qompl->as<ob::SE2StateSpaceFullInterpolate::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  q(0)=qomplSE2->getX();
  q(1)=qomplSE2->getY();
  q(3)=qomplSE2->getYaw();
  return q;

}

void GeometricCSpaceOMPLRCONTACT::print() const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            : " << 3 << std::endl;

  ob::SE2StateSpaceFullInterpolate *cspace = this->space->as<ob::SE2StateSpaceFullInterpolate>();

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
