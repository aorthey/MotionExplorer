#include "planner/cspace/cspace_geometric_empty.h"
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/EmptyStateSpace.h>


GeometricCSpaceOMPLEmpty::GeometricCSpaceOMPLEmpty(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLEmpty::initSpace()
{
  ob::StateSpacePtr RN = std::make_shared<ob::EmptyStateSpace>();
  this->space = RN;
}
uint GeometricCSpaceOMPLEmpty::GetDimensionality() const{
  return 0;
}

void GeometricCSpaceOMPLEmpty::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
}

Config GeometricCSpaceOMPLEmpty::OMPLStateToConfig(const ob::State *qompl)
{
  Config q;q.resize(robot->q.size());q.setZero();
  return q;
}
void GeometricCSpaceOMPLEmpty::print(std::ostream& out) const
{
  out << std::string(80, '-') << std::endl;
  out << "Empty ConfigurationSpace" << std::endl;
  out << std::string(80, '-') << std::endl;
}


Vector3 GeometricCSpaceOMPLEmpty::getXYZ(const ob::State *s)
{
  Vector3 q(0,0,0);
  return q;
}
