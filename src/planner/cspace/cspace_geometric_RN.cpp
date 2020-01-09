#include "planner/cspace/cspace_geometric_RN.h"

#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLRN::GeometricCSpaceOMPLRN(RobotWorld *world_, int robot_idx, int dimension):
  GeometricCSpaceOMPL(world_, robot_idx), N(dimension)
{
}

void GeometricCSpaceOMPLRN::initSpace()
{
  ob::StateSpacePtr RN = (std::make_shared<ob::RealVectorStateSpace>(N));
  this->space = RN;
  ob::RealVectorStateSpace *cspace = this->space->as<ob::RealVectorStateSpace>();

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
  cspace->setBounds(bounds);
  bounds.check();

}

void GeometricCSpaceOMPLRN::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  for(uint k = 0; k < N; k++){
    qomplRN->values[k] = q(k);
  }
}

Config GeometricCSpaceOMPLRN::OMPLStateToConfig(const ob::State *qompl){

  const ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();

  Config q;q.resize(robot->q.size());q.setZero();
  for(uint k = 0; k < N; k++){
    q(k) = qomplRN->values[k];
  }
  return q;

}
void GeometricCSpaceOMPLRN::print(std::ostream& out) const
{
  out << std::string(80, '-') << std::endl;
  out << "RealVectorStateSpace ";
  out << "(Robot: " << robot->name << ")" << std::endl;
  out << "Dimensionality : " 
    << GetDimensionality() << "[OMPL] and "
    << robot->q.size() << "[KLAMPT]" << std::endl;

  ob::RealVectorStateSpace *cspace = space->as<ob::RealVectorStateSpace>();

  const ob::RealVectorBounds bounds = cspace->getBounds();
  std::vector<double> min = bounds.low;
  std::vector<double> max = bounds.high;
  out << "RN bounds min     : ";
  for(uint i = 0; i < min.size(); i++){
    out << " " << min.at(i);
  }
  out << std::endl;

  out << "RN bounds max     : ";
  for(uint i = 0; i < max.size(); i++){
    out << " " << max.at(i);
  }
  out << std::endl;

  out << std::string(80, '-') << std::endl;
}


Vector3 GeometricCSpaceOMPLRN::getXYZ(const ob::State *s)
{
  const ob::RealVectorStateSpace::StateType *qomplRN = s->as<ob::RealVectorStateSpace::StateType>();

  double x = qomplRN->values[0];
  double y = 0;
  double z = 0;

  if(N>1) y = qomplRN->values[1];
  if(N>2) z = qomplRN->values[2];
  Vector3 q(x,y,z);
  return q;
}
