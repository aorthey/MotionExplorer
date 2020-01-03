#include "planner/cspace/cspace_geometric_SO2.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "ompl/base/spaces/SO2StateSpaceFullInterpolate.h"

GeometricCSpaceOMPLSO2::GeometricCSpaceOMPLSO2(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLSO2::initSpace()
{
  //std::cout << "[CSPACE] Robot \"" << robot->name << "\" Configuration Space: SE(2)" << std::endl;

  ob::StateSpacePtr SO2 = (std::make_shared<ob::SO2StateSpaceFullInterpolate>());
  this->space = SO2;
  //ob::SO2StateSpaceFullInterpolate *cspace = this->space->as<ob::SO2StateSpaceFullInterpolate>();
}

void GeometricCSpaceOMPLSO2::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2 = qompl->as<ob::SO2StateSpaceFullInterpolate::StateType>();

  qomplSO2->value = q(6);
}

Config GeometricCSpaceOMPLSO2::OMPLStateToConfig(const ob::State *qompl){

  const ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2 = qompl->as<ob::SO2StateSpaceFullInterpolate::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  q(6)=qomplSO2->value;
  return q;

}

void GeometricCSpaceOMPLSO2::print(std::ostream& out) const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            : " << 1 << std::endl;
}
