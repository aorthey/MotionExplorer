#include "planner/cspace/cspace_geometric_R3S2.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "ompl/base/spaces/SO2StateSpaceFullInterpolate.h"

GeometricCSpaceOMPLR3S2::GeometricCSpaceOMPLR3S2(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLR3S2::initSpace()
{
  ob::StateSpacePtr R3 = (std::make_shared<ob::RealVectorStateSpace>(3));
  ob::StateSpacePtr S1a = (std::make_shared<ob::SO2StateSpaceFullInterpolate>());
  ob::StateSpacePtr S1b = (std::make_shared<ob::SO2StateSpaceFullInterpolate>());
  this->space = R3 + S1a + S1b;
  ob::RealVectorStateSpace *cspaceR3 = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0);

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> low;
  vector<double> high;
  for(uint k = 0; k < 3; k++){
    low.push_back(minimum.at(k));
    high.push_back(maximum.at(k));
  }

  ob::RealVectorBounds bounds(3);
  bounds.low = low;
  bounds.high = high;
  cspaceR3->setBounds(bounds);
  bounds.check();
}

void GeometricCSpaceOMPLR3S2::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2SpaceA = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpaceFullInterpolate::StateType>(1);
  ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2SpaceB = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpaceFullInterpolate::StateType>(2);
  // static_cast<ob::SO2StateSpaceFullInterpolate::StateType*>(qomplSO2SpaceA)->value = q(3);
  // static_cast<ob::SO2StateSpaceFullInterpolate::StateType*>(qomplSO2SpaceB)->value = q(4);
  qomplSO2SpaceA->value = q(3);
  qomplSO2SpaceB->value = q(4);

  for(uint k = 0; k < 3; k++){
    qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[k] = q(k);
  }
}

Config GeometricCSpaceOMPLR3S2::OMPLStateToConfig(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplR3Space = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  const ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2SpaceA = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpaceFullInterpolate::StateType>(1);
  const ob::SO2StateSpaceFullInterpolate::StateType *qomplSO2SpaceB = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpaceFullInterpolate::StateType>(2);

  Config q;q.resize(6);q.setZero();
  for(uint k = 0; k < 3 ; k++){
    q(k) = qomplR3Space->values[k];
  }
  q(3) = qomplSO2SpaceA->value;
  q(4) = qomplSO2SpaceB->value;
  q(5)=0.0;

  return q;
}
Vector3 GeometricCSpaceOMPLR3S2::getXYZ(const ob::State *s)
{
  OMPL_ERROR("NYI");
  throw "NYI";
}
