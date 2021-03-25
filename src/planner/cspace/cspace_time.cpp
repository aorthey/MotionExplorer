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
  T->setBounds(input.timeLB, input.timeUB);
  this->space = T;

  // externalRobotsIdxs = input.externalRobotsIdxs;
  // externalRobotsPathFiles = input.externalRobotsPathFiles;

  // for(uint k = 0; k < externalRobotsIdxs.size(); k++)
  // {
  //   int id = externalRobotsIdxs.at(k);
  //   Robot *robot = world->robots[id];
  //   std::string file = externalRobotsPathFiles.at(k);
  //   PathPiecewiseLinearPtr path = std::make_shared<PathPiecewiseLinear>(this);

  //   std::cout << "reading in robot id:" << id << " file: " << file << std::endl;
  //   std::cout << robot->name << std::endl;
  // }
  // exit(0);

  //add here robots which move along time
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

Config CSpaceOMPLTime::OMPLStateToConfig(const ob::State *qompl)
{

  // const ob::TimeStateSpace::StateType *qOMPLTime = qompl->as<ob::TimeStateSpace::StateType>(1);

  Config q;q.resize(0);q.setZero();
  return q;
}

//void CSpaceOMPLTime::UpdateEnvironmentFromState(const ob::State* state)
//{
//  double t = GetTime(state);
//  for(uint k = 0; k < externalRobotsIdxs.size(); k++)
//  {
//    int idx = externalRobotsIdxs.at(k); 
//    std::cout << idx << std::endl;
//    //PathPiecewiseLinearPtr pwl = pathsRobots_.at(k);
//    ////Get configuration
//    //Config q = pwl->Eval(t);
//    //Robot* robot = world->robots[idx];
//    //std::cout << "Set robot " << robot->name << " to " << q << std::endl;
//    //robot->UpdateConfig(q);
//    //robot->UpdateGeometry();
//  }
//}
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
