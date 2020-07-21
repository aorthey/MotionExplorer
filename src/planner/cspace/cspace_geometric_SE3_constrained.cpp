#include "planner/cspace/cspace_geometric_SE3_constrained.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLSE3Constrained::GeometricCSpaceOMPLSE3Constrained(RobotWorld *world_, int robot_idx):
  BaseT(world_, robot_idx)
{
}
bool GeometricCSpaceOMPLSE3Constrained::isDynamic() const
{
  return false;
}

void GeometricCSpaceOMPLSE3Constrained::initSpace()
{
  ob::StateSpacePtr R3(std::make_shared<ob::RealVectorStateSpace>(3));
  ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
  ob::StateSpacePtr R2(std::make_shared<ob::RealVectorStateSpace>(2));

  //###########################################################################
  // Set bounds
  //###########################################################################
  std::vector<double> minimum, maximum;
  //RPY
  minimum = robot->qMin;
  maximum = robot->qMax;

  ob::RealVectorBounds boundsR3(3);
  for(uint k = 0; k < 3; k++){
    boundsR3.low[k] = minimum.at(k);
    boundsR3.high[k] = maximum.at(k);
  }
  static_pointer_cast<ob::RealVectorStateSpace>(R3)->setBounds(boundsR3);

  ob::RealVectorBounds boundsR2(2);
  for(uint k = 4; k < 6; k++){
    boundsR2.low[k-4] = minimum.at(k);
    boundsR2.high[k-4] = maximum.at(k);
  }
  static_pointer_cast<ob::RealVectorStateSpace>(R2)->setBounds(boundsR2);

  this->space = R3 + SO2 + R2;
}

void GeometricCSpaceOMPLSE3Constrained::print(std::ostream& out) const
{
}

void GeometricCSpaceOMPLSE3Constrained::ConfigToOMPLState(const Config &q, ob::State *qompl)
{

  ob::RealVectorStateSpace::StateType *qomplR3 = 
    qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  ob::SO2StateSpace::StateType *qomplSO2 = 
    qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
  ob::RealVectorStateSpace::StateType *qomplR2 = 
    qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

  for(uint k = 0; k < 3; k++){
    qomplR3->values[k] = q[k];
  }
  qomplSO2->value = q[3];
  qomplR2->values[0] = q[4];
  qomplR2->values[1] = q[5];
}

Config GeometricCSpaceOMPLSE3Constrained::OMPLStateToConfig(const ob::State *qompl)
{
  const ob::RealVectorStateSpace::StateType *qomplR3 = 
    qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  const ob::SO2StateSpace::StateType *qomplSO2 = 
    qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
  const ob::RealVectorStateSpace::StateType *qomplR2 = 
    qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);


  Config q;
  q.resize(6+Nklampt);
  q.setZero();

  for(uint k = 0; k < 3; k++){
    q[k] = qomplR3->values[k];
  }

  q[3] = qomplSO2->value;
  q[4] = qomplR2->values[0];
  q[5] = qomplR2->values[1];

  return q;
}

Vector3 GeometricCSpaceOMPLSE3Constrained::getXYZ(const ob::State *s)
{
  const ob::RealVectorStateSpace::StateType *R3 = 
    s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);

  double x = R3->values[0];
  double y = R3->values[1];
  double z = R3->values[2];
  Vector3 q(x,y,z);
  return q;
}
