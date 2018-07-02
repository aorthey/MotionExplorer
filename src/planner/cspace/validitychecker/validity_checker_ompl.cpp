#include "planner/cspace/validitychecker/validity_checker_ompl.h"

OMPLValidityChecker::OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpace *inner_):
  ob::StateValidityChecker(si), cspace(cspace_), inner(inner_)
{
}

CSpaceOMPL* OMPLValidityChecker::GetCSpacePtr() const
{
  return cspace;
}
bool OMPLValidityChecker::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = cspace->OMPLStateToConfig(state);
  SingleRobotCSpace* csi = static_cast<SingleRobotCSpace*>(inner);
  return IsCollisionFree(csi, q);
}

double OMPLValidityChecker::Distance(const ob::State* state) const
{
  Config q = cspace->OMPLStateToConfig(state);
  SingleRobotCSpace* space = static_cast<SingleRobotCSpace*>(inner);
  Robot* robot = space->GetRobot();
  robot->UpdateConfig(q);
  robot->UpdateGeometry();

  int id = space->world.RobotID(space->index);
  vector<int> idrobot(1,id);
  vector<int> idothers;
  for(size_t i=0;i<space->world.terrains.size();i++)
  {
    idothers.push_back(space->world.TerrainID(i));
  }
  for(size_t i=0;i<space->world.rigidObjects.size();i++)
  {
    idothers.push_back(space->world.RigidObjectID(i));
  }

  pair<int,int> res;

  //selfcollision checking
  res = space->settings->CheckCollision(space->world,idrobot);
  if(res.first >= 0) return 0;
  //environment collision checking
  res = space->settings->CheckCollision(space->world,idrobot,idothers);
  if(res.first >= 0) return 0;

  int closest1, closest2;
  double d = space->settings->DistanceLowerBound(space->world, idrobot, idothers, 0, dInf, &closest1, &closest2);

  return d;
}


//same as Singlerobotcspace but ignore other robots (because QS requires
//multiple nested robots)
bool OMPLValidityChecker::IsCollisionFree(SingleRobotCSpace *space, Config q) const{
  Robot* robot = space->GetRobot();
  robot->UpdateConfig(q);
  robot->UpdateGeometry();

  int id = space->world.RobotID(space->index);
  vector<int> idrobot(1,id);
  vector<int> idothers;
  for(size_t i=0;i<space->world.terrains.size();i++)
    idothers.push_back(space->world.TerrainID(i));
  for(size_t i=0;i<space->world.rigidObjects.size();i++)
    idothers.push_back(space->world.RigidObjectID(i));

  pair<int,int> res;

  //selfcollision checking
  res = space->settings->CheckCollision(space->world,idrobot);
  if(res.first >= 0) return false;
  //environment collision checking
  res = space->settings->CheckCollision(space->world,idrobot,idothers);

  if(res.first >= 0) return false;

  return true;
}
bool OMPLValidityChecker::IsNecessary(const ob::State* state) const
{
  return isValid(state);
}
bool OMPLValidityChecker::IsSufficient(const ob::State* state) const
{
  static bool first = true;
  if(first){
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "[WARNING] enableSufficiency not activated in XML file" << std::endl;
    std::cout << std::string(80, '#') << std::endl;
    first = !first;
  }
  return false;
}


OMPLValidityCheckerInnerOuter::OMPLValidityCheckerInnerOuter(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpace *inner_, CSpace *outer_):
  OMPLValidityChecker(si, cspace_, inner_), outer(outer_)
{
}

bool OMPLValidityCheckerInnerOuter::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = cspace->OMPLStateToConfig(state);
  SingleRobotCSpace* csi = static_cast<SingleRobotCSpace*>(inner);
  SingleRobotCSpace* cso = static_cast<SingleRobotCSpace*>(outer);
  return IsCollisionFree(csi, q) && (!IsCollisionFree(cso,q));
}

OMPLValidityCheckerNecessarySufficient::OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpace *outer_):
  OMPLValidityChecker(si, cspace_, cspace_->GetCSpacePtr()), outer(outer_)
{
}

bool OMPLValidityCheckerNecessarySufficient::IsSufficient(const ob::State* state) const
{
  Config q = cspace->OMPLStateToConfig(state);
  SingleRobotCSpace* cso = static_cast<SingleRobotCSpace*>(outer);
  return IsCollisionFree(cso, q);
}
