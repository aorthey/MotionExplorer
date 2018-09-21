#include "neighborhood_RN.h"
#include "neighborhood_SE2.h"
#include "neighborhood_SE3.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/StateSpaceTypes.h>

OMPLValidityChecker::OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_):
  ob::StateValidityChecker(si), cspace(cspace_)
{
  klampt_single_robot_cspace = static_cast<SingleRobotCSpace*>(cspace_->GetCSpaceKlamptPtr());

  //set neighborhood
  //STATE_SPACE_UNKNOWN = 0,
  //STATE_SPACE_REAL_VECTOR = 1,
  //STATE_SPACE_SO2 = 2,
  //STATE_SPACE_SO3 = 3,
  //STATE_SPACE_SE2 = 4,
  //STATE_SPACE_SE3 = 5,
  //STATE_SPACE_TIME = 6,
  //STATE_SPACE_DISCRETE = 7,
  int space_type = cspace->SpaceInformationPtr()->getStateSpace()->getType();
  if(cspace->isFreeFloating()){
    uint n = cspace->GetDimensionality();
    std::cout << std::string(80, '-') << std::endl;
    if(space_type == ob::STATE_SPACE_REAL_VECTOR && n<=3){
      //rotational invariant rigid object
      std::cout << "R" << n << " neighborhood" << std::endl;
      neighborhood = new NeighborhoodRN();
    }else if(space_type == ob::STATE_SPACE_SE2){
      std::cout << "SE2 neighborhood" << std::endl;
      neighborhood = new NeighborhoodSE2();
    }else if(space_type == ob::STATE_SPACE_SE3){
      std::cout << "SE3 neighborhood" << std::endl;
      neighborhood = new NeighborhoodSE3();
    }else{
      std::cout << "free Floating" << std::endl;
      std::cout << "Do not know how to compute a neighborhood for type " << space_type << std::endl;
      exit(0);
    }
  }else{
    if(space_type==ob::STATE_SPACE_REAL_VECTOR){
      std::cout << "Do not know how to compute a neighborhood for type " << space_type << std::endl;
      std::cout << "NYI" << std::endl;
      exit(0);
    }else{
      std::cout << "Do not know how to compute a neighborhood for type " << space_type << std::endl;
      exit(0);
    }

  }
  neighborhood->Init();

}

CSpaceOMPL* OMPLValidityChecker::GetCSpaceOMPLPtr() const
{
  return cspace;
}
bool OMPLValidityChecker::isValid(const ob::State* state) const
{
  Config q = cspace->OMPLStateToConfig(state);
  return IsCollisionFree(klampt_single_robot_cspace, q) && si_->satisfiesBounds(state);
}

double OMPLValidityChecker::SufficientDistance(const ob::State* state) const
{
  return 0;
}

double OMPLValidityChecker::Distance(const ob::State* state) const
{
  return DistanceToRobot(state, klampt_single_robot_cspace);
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

  return neighborhood->WorkspaceDistanceToConfigurationSpaceDistance(d);
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

// OMPLValidityCheckerInnerOuter::OMPLValidityCheckerInnerOuter(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpace *inner_, CSpace *outer_):
//   OMPLValidityChecker(si, cspace_, inner_), outer(outer_)
// {
// }

// bool OMPLValidityCheckerInnerOuter::isValid(const ob::State* state) const
// {
//   const ob::StateSpacePtr ssp = si_->getStateSpace();
//   Config q = cspace->OMPLStateToConfig(state);
//   SingleRobotCSpace* csi = static_cast<SingleRobotCSpace*>(inner);
//   SingleRobotCSpace* cso = static_cast<SingleRobotCSpace*>(outer);
//   return IsCollisionFree(csi, q) && (!IsCollisionFree(cso,q));
// }

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
  return neighborhood->WorkspaceDistanceToConfigurationSpaceDistance(dw);
}
