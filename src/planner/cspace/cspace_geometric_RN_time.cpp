#include "planner/cspace/cspace_geometric_RN_time.h"

#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

GeometricCSpaceOMPLRNTime::GeometricCSpaceOMPLRNTime(RobotWorld *world_, int robot_idx, int dimension):
  GeometricCSpaceOMPL(world_, robot_idx), N(dimension)
{
}

void GeometricCSpaceOMPLRNTime::initSpace()
{
  //last dimension is TIME
  ob::StateSpacePtr RN = (std::make_shared<ob::RealVectorStateSpace>(N));
  ob::StateSpacePtr T = (std::make_shared<ob::TimeStateSpace>());
  this->space = RN + T;

  ob::RealVectorStateSpace *cspaceRN = 
    this->space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0);

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> low;
  vector<double> high;
  for(uint k = 0; k < N; k++){
    low.push_back(minimum.at(k));
    high.push_back(maximum.at(k));
  }

  ob::RealVectorBounds bounds(N);
  bounds.low = low;
  bounds.high = high;
  cspaceRN->setBounds(bounds);
  bounds.check();

  ob::TimeStateSpace *cspaceT = this->space->as<ob::CompoundStateSpace>()->as<ob::TimeStateSpace>(1);
  cspaceT->setBounds(0,1);
}

void GeometricCSpaceOMPLRNTime::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::RealVectorStateSpace::StateType *qOMPLRN = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  ob::TimeStateSpace::StateType *qOMPLTime = qompl->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1);
  for(uint k = 0; k < N; k++){
    qOMPLRN->values[k] = q(k);
  }
  qOMPLTime->position = q(N);
}

Config GeometricCSpaceOMPLRNTime::OMPLStateToConfig(const ob::State *qompl){

  const ob::RealVectorStateSpace::StateType *qOMPLRN = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
  const ob::TimeStateSpace::StateType *qOMPLTime = qompl->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1);

  Config q;q.resize(robot->q.size()+1);q.setZero();
  for(uint k = 0; k < N; k++){
    q(k) = qOMPLRN->values[k];
  }
  q(N) = qOMPLTime->position;
  return q;

}

bool GeometricCSpaceOMPLRNTime::isTimeDependent()
{
  return true;
}

double GeometricCSpaceOMPLRNTime::GetTime(const ob::State *qompl)
{
  const ob::TimeStateSpace::StateType *qOMPLTime = 
    qompl->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1);
  double t = qOMPLTime->position;
  return t;
}

void GeometricCSpaceOMPLRNTime::print(std::ostream& out) const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL CSPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            : R^" << GetDimensionality() << std::endl;

  ob::RealVectorStateSpace *cspace = 
    space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(0);

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

Vector3 GeometricCSpaceOMPLRNTime::getXYZ(const ob::State *s)
{
  // double t = GetTime(s);
  OMPL_ERROR("NYI");
  throw "NYI";
}
