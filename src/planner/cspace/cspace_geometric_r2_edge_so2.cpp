#include "planner/cspace/cspace_geometric_r2_edge_so2.h"
#include "planner/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

GeometricCSpaceOMPLR2EdgeSO2::GeometricCSpaceOMPLR2EdgeSO2(Robot *robot_, CSpace *space_, const Config& q_src, const Config& q_trg):
  GeometricCSpaceOMPL(robot_, space_),
  q1(q_src), q2(q_trg)
{
  double x1 = q1(0);
  double y1 = q1(1);
  double x2 = q2(0);
  double y2 = q2(1);

  edge_length = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

void GeometricCSpaceOMPLR2EdgeSO2::initSpace()
{
  ob::StateSpacePtr R = (std::make_shared<ob::RealVectorStateSpace>(1));
  ob::StateSpacePtr S1 = (std::make_shared<ob::SO2StateSpace>());
  this->space = R + S1;
  ob::RealVectorStateSpace *cspaceR = this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0);
  ob::SO2StateSpace *cspaceS1 = this->space->as<ob::CompoundStateSpace>()->as<ob::SO2StateSpace>(1);
  ob::RealVectorBounds cbounds(1);
  cbounds.setLow(0);
  cbounds.setHigh(1+1e-10);
  cspaceR->setBounds(cbounds);

}
void GeometricCSpaceOMPLR2EdgeSO2::print()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "[CSpace]" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Name                      : OMPL CSPACE R^2 Edge SO(2)" << std::endl;
  std::cout << "Dimensionality Space      :" << GetDimensionality() << std::endl;
}

ob::ScopedState<> GeometricCSpaceOMPLR2EdgeSO2::ConfigToOMPLState(const Config &q){

  ob::ScopedState<> qompl(space);
  ob::SO2StateSpace::StateType *qomplSO2Space = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);
  static_cast<ob::SO2StateSpace::StateType*>(qomplSO2Space)->value = q(3);

  double x = q(0) - q1(0);
  double y = q(1) - q1(1);
  double length = sqrt((x*x)+(y*y));

  double t = length / edge_length;

  qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = t;
  return qompl;
}

Config GeometricCSpaceOMPLR2EdgeSO2::OMPLStateToConfig(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplR2Space = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  const ob::SO2StateSpace::StateType *qomplSO2Space = qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);

  double t = qomplR2Space->values[0];
  double qyaw = qomplSO2Space->value;
  Config qedge = q1 + t*(q2-q1);

  Config q;q.resize(robot->q.size());q.setZero();
  q(0)=qedge(0);
  q(1)=qedge(1);
  q(3)=qyaw;

  return q;

}
