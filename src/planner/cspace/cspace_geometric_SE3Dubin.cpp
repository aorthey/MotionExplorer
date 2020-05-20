#include "planner/cspace/cspace_geometric_SE3Dubin.h"
#include <ompl/base/spaces/DubinsAirplaneStateSpace.h>

GeometricCSpaceOMPLSE3Dubin::GeometricCSpaceOMPLSE3Dubin(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
    turningRadius_ = 0.5;
    climbingAngle_ = 0.4;
}

void GeometricCSpaceOMPLSE3Dubin::initSpace()
{
  ob::StateSpacePtr SE3Dubin(std::make_shared<ob::DubinsAirplaneStateSpace>(
        turningRadius_, climbingAngle_));
  ob::DubinsAirplaneStateSpace *cspaceDubin;

  this->space = SE3Dubin;
  cspaceDubin = this->space->as<ob::DubinsAirplaneStateSpace>();

  //###########################################################################
  // Set bounds
  //###########################################################################
  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  assert(minimum.size() == 6+Nklampt);
  assert(maximum.size() == 6+Nklampt);

  vector<double> lowSE3;
  lowSE3.push_back(minimum.at(0));
  lowSE3.push_back(minimum.at(1));
  lowSE3.push_back(minimum.at(2));
  vector<double> highSE3;
  highSE3.push_back(maximum.at(0));
  highSE3.push_back(maximum.at(1));
  highSE3.push_back(maximum.at(2));

  ob::RealVectorBounds boundsSE3(3);
  boundsSE3.low = lowSE3;
  boundsSE3.high = highSE3;
  cspaceDubin->setBounds(boundsSE3);
  boundsSE3.check();
}

void GeometricCSpaceOMPLSE3Dubin::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SimpleSE3StateSpace::StateType *qomplDubin = qompl->as<ob::DubinsAirplaneStateSpace::StateType>();
  qomplDubin->setXYZ(q(0),q(1),q(2));
  qomplDubin->setYaw(q(3));
}

Config GeometricCSpaceOMPLSE3Dubin::OMPLStateToConfig(const ob::State *qompl)
{

  const ob::SimpleSE3StateSpace::StateType *qomplDubin = qompl->as<ob::DubinsAirplaneStateSpace::StateType>();
  Config q;
  q.resize(6+Nklampt);
  q.setZero();

  q(0) = qomplDubin->getX();
  q(1) = qomplDubin->getY();
  q(2) = qomplDubin->getZ();
  q(3) = qomplDubin->getYaw();

  return q;
}
ob::SpaceInformationPtr GeometricCSpaceOMPLSE3Dubin::SpaceInformationPtr()
{
    si = BaseT::SpaceInformationPtr();
    si->setMotionValidator(std::make_shared<ob::DubinsAirplaneMotionValidator>(si));
    return si;
}
