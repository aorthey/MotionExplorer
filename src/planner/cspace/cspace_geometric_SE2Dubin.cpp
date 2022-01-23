#include "planner/cspace/cspace_geometric_SE2Dubin.h"
#include <ompl/base/spaces/DubinsStateSpace.h>

GeometricCSpaceOMPLSE2Dubin::GeometricCSpaceOMPLSE2Dubin(RobotWorld *world_, int robot_idx):
  BaseT(world_, robot_idx)
{
    turningRadius_ = 0.2;
}

void GeometricCSpaceOMPLSE2Dubin::initSpace()
{

  ob::StateSpacePtr SE2Dubin(std::make_shared<ob::DubinsStateSpace>(turningRadius_, false));
  ob::DubinsStateSpace *cspaceDubin;
  this->space = SE2Dubin;
  cspaceDubin = this->space->as<ob::DubinsStateSpace>();

  //###########################################################################
  // Set bounds
  //###########################################################################

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
  cspaceDubin->setBounds(bounds);
  bounds.check();
}

void GeometricCSpaceOMPLSE2Dubin::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  ob::SE2StateSpace::StateType *qomplDubin = qompl->as<ob::DubinsStateSpace::StateType>();
  qomplDubin->setXY(q(0),q(1));
  qomplDubin->setYaw(q(3));
}

Config GeometricCSpaceOMPLSE2Dubin::OMPLStateToConfig(const ob::State *qompl){
  const ob::SE2StateSpace::StateType *qomplDubins = qompl->as<ob::SE2StateSpace::StateType>();
  Config q;q.resize(robot->q.size());q.setZero();
  q(0)=qomplDubins->getX();
  q(1)=qomplDubins->getY();
  q(3)=qomplDubins->getYaw();

  return q;
}
ob::SpaceInformationPtr GeometricCSpaceOMPLSE2Dubin::SpaceInformationPtr()
{
    si = BaseT::SpaceInformationPtr();
    // si->setMotionValidator(std::make_shared<ob::DubinsMotionValidator>(si));
    return si;
}
