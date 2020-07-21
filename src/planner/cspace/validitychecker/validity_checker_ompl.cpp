#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "common.h"
#include <ompl/base/StateSpaceTypes.h>

OMPLValidityChecker::OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_):
  ob::StateValidityChecker(si), cspace(cspace_)
{
    klampt_single_robot_cspace = static_cast<SingleRobotCSpace*>(cspace_->GetCSpaceKlamptPtr());
    specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::BOUNDED_APPROXIMATE;
}


void OMPLValidityChecker::SetNeighborhood(double cspace_constant)
{
  neighborhood = new Neighborhood(cspace_constant);
}

CSpaceOMPL* OMPLValidityChecker::GetCSpaceOMPLPtr() const
{
  return cspace;
}
bool OMPLValidityChecker::isValid(const ob::State* state) const
{
  Config q = cspace->OMPLStateToConfig(state);
  if(cspace->isTimeDependent()){
    cspace->GetTime(state);
  }
  return IsCollisionFree(klampt_single_robot_cspace, q) && si_->satisfiesBounds(state);
}

bool OMPLValidityChecker::operator ==(const ob::StateValidityChecker &rhs) const
{
  const OMPLValidityChecker &vrhs = static_cast<const OMPLValidityChecker&>(rhs);
  int idx_lhs = GetCSpaceOMPLPtr()->GetRobotIndex();
  int idx_rhs = vrhs.GetCSpaceOMPLPtr()->GetRobotIndex();
  return (idx_lhs == idx_rhs);
}

double OMPLValidityChecker::SufficientDistance(const ob::State* state) const
{
  return 0;
}

double OMPLValidityChecker::clearance(const ob::State* state) const
{
  double c = DistanceToRobot(state, klampt_single_robot_cspace);
  return 1.0/c;
}

double OMPLValidityChecker::DistanceToRobot(const ob::State* state, SingleRobotCSpace *space) const
{
  Config q = cspace->OMPLStateToConfig(state);
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
  // return neighborhood->WorkspaceDistanceToConfigurationSpaceDistance(d);
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
bool OMPLValidityChecker::IsFeasible(const ob::State* state) const
{
  return isValid(state);
}
bool OMPLValidityChecker::IsSufficientFeasible(const ob::State* state) const
{
  return false;
}

OMPLValidityCheckerNecessarySufficient::OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpaceKlampt *outer_):
  OMPLValidityChecker(si, cspace_)
{
  klampt_single_robot_cspace_outer_approximation = static_cast<SingleRobotCSpace*>(outer_); 
}

bool OMPLValidityCheckerNecessarySufficient::IsSufficientFeasible(const ob::State* state) const
{
  Config q = cspace->OMPLStateToConfig(state);
  return IsCollisionFree(klampt_single_robot_cspace_outer_approximation, q);
}

double OMPLValidityCheckerNecessarySufficient::SufficientDistance(const ob::State* state) const
{
  double dw = DistanceToRobot(state, klampt_single_robot_cspace_outer_approximation);
  return dw;
  // return neighborhood->WorkspaceDistanceToConfigurationSpaceDistance(dw);
}
