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

uint GeometricCSpaceOMPLEmpty::GetDimensionality() const
{
  return 0;
}

uint GeometricCSpaceOMPLEmpty::GetKlamptDimensionality() const
{
  if(GetRobotIndex() >= 0) return BaseT::GetKlamptDimensionality();
  else return 0;
}

void GeometricCSpaceOMPLEmpty::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  q_ = q;
}

Config GeometricCSpaceOMPLEmpty::OMPLStateToConfig(const ob::State *qompl)
{
  if(GetRobotIndex() >= 0)
  {
      if(q_.size() != robot->q.size()) 
      {
        q_.resize(robot->q.size());q_.setZero();
      }
  }else
  {
      q_.resize(0);q_.setZero();
  }
  return q_;
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
