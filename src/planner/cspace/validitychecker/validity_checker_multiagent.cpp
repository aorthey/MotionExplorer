#include "planner/cspace/validitychecker/validity_checker_multiagent.h"
#include "common.h"
#include <ompl/base/StateSpaceTypes.h>

OMPLValidityCheckerMultiAgent::OMPLValidityCheckerMultiAgent(const ob::SpaceInformationPtr &si, 
    CSpaceOMPLMultiAgent *cspace, std::vector<CSpaceOMPL*> cspaces):
  ob::StateValidityChecker(si), cspace_(cspace), cspaces_(cspaces)
{
  for(uint k = 0; k < cspaces_.size(); k++){
    SingleRobotCSpace *kck = static_cast<SingleRobotCSpace*>(cspaces.at(k)->GetCSpaceKlamptPtr());
    klampt_single_robot_cspaces_.push_back(kck);
  }
}
bool OMPLValidityCheckerMultiAgent::isValid(const ob::State* state) const
{
  if(!cspace_->SatisfiesBounds(state)) return false;

  Config q = cspace_->OMPLStateToConfig(state);
  cspace_->UpdateRobotConfig(q);

  std::vector<int> ridxs = cspace_->GetRobotIdxs();

  for(uint k = 0; k < ridxs.size(); k++){
    SingleRobotCSpace* space = klampt_single_robot_cspaces_.at(k);

    int rk = ridxs.at(k);
    vector<int> idrobot(1, rk);

    vector<int> idothers;
    for(size_t i=0;i<space->world.terrains.size();i++)
      idothers.push_back(space->world.TerrainID(i));
    for(size_t i=0;i<space->world.rigidObjects.size();i++)
      idothers.push_back(space->world.RigidObjectID(i));
    for(size_t i=0;i<space->world.robots.size();i++)
    {
      if((uint)rk != i){
        idothers.push_back(space->world.RobotID(i));
      }
    }

    // if(!space->settings->CheckCollision(space->world, -1)) return false;

    pair<int,int> res;
    res = space->settings->CheckCollision(space->world, idrobot);
    if(res.first >= 0) return false;
    res = space->settings->CheckCollision(space->world, idrobot, idothers);
    if(res.first >= 0) return false;

  }
  return true;
}

