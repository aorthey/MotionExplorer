#include "planner/cspace/cspace_time.h"

#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

CSpaceOMPLTime::CSpaceOMPLTime(RobotWorld *world_):
  CSpaceOMPL(world_, -1)
{
}

void CSpaceOMPLTime::initSpace()
{
  auto T = (std::make_shared<ob::TimeStateSpace>());
  T->setBounds(0,2);
  this->space = T;
}

void CSpaceOMPLTime::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
}

bool CSpaceOMPLTime::isDynamic() const
{
  return false;
}

uint CSpaceOMPLTime::GetKlamptDimensionality() const
{
  return 0;
}

Config CSpaceOMPLTime::OMPLStateToConfig(const ob::State *qompl){

  // const ob::TimeStateSpace::StateType *qOMPLTime = qompl->as<ob::TimeStateSpace::StateType>(1);

  Config q;q.resize(0);q.setZero();
  return q;
}

double CSpaceOMPLTime::GetTime(const ob::State *qompl)
{
  const ob::TimeStateSpace::StateType *qOMPLTime = 
    qompl->as<ob::TimeStateSpace::StateType>();
  double t = qOMPLTime->position;
  return t;
}

void CSpaceOMPLTime::SetTime(ob::State *qompl, double time)
{
  ob::TimeStateSpace::StateType *qOMPLTime = 
    qompl->as<ob::TimeStateSpace::StateType>();
  qOMPLTime->position = time;
}

void CSpaceOMPLTime::print(std::ostream& out) const
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "OMPL TIME STATE SPACE" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

Vector3 CSpaceOMPLTime::getXYZ(const ob::State *s)
{
  return Vector3(0,0,0);
}
