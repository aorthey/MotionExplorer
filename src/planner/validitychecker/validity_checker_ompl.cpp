#include "planner/validitychecker/validity_checker_ompl.h"

OMPLValidityChecker::OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_):
  ob::StateValidityChecker(si), ompl_space(ompl_space_), inner(inner_)
{
}

bool OMPLValidityChecker::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = ompl_space->OMPLStateToConfig(state);
  SingleRobotCSpace* csi = static_cast<SingleRobotCSpace*>(inner);
  return IsCollisionFree(csi, q);
}
void GetGeometries2(RobotWorld& world,const vector<int>& ids,vector<Geometry::AnyCollisionGeometry3D*>& geoms,vector<int>& activeids)
{
  geoms.reserve(ids.size());
  activeids.reserve(ids.size());
  for(size_t i=0;i<ids.size();i++) {
    int robotindex = world.IsRobot(ids[i]);;
    if(robotindex >=0) {
      //crud, have to expand
      Robot* robot = world.robots[robotindex];
      for(size_t j=0;j<robot->links.size();j++) {
  Geometry::AnyCollisionGeometry3D* g=robot->geometry[j];
  if(g && !g->Empty()) {
    geoms.push_back(g);
    activeids.push_back(world.RobotLinkID(robotindex,j));
  }
      }
    }
    else {
      Geometry::AnyCollisionGeometry3D* g=world.GetGeometry(ids[i]);
      if(g && !g->Empty()) {
  geoms.push_back(g);
  activeids.push_back(ids[i]);
      }
    }
  }
}


double OMPLValidityChecker::Distance(const ob::State* state) const
{
  Config q = ompl_space->OMPLStateToConfig(state);
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
  ////------------------------------->
  //std::vector<Geometry::AnyCollisionGeometry3D*> geoms1,geoms2;
  //vector<int> activeids1,activeids2;

  //GetGeometries2(space->world,idrobot,geoms1,activeids1);
  //GetGeometries2(space->world,idothers,geoms2,activeids2);

  //vector<AABB3D> bbs1(geoms1.size());
  //vector<AABB3D> bbs2(geoms2.size());
  //for(size_t i=0;i<geoms1.size();i++) 
  //{
  //  bbs1[i]=geoms1[i]->GetAABB();
  //  std::cout << bbs1[i].bmin << std::endl;
  //  std::cout << bbs1[i].bmax << std::endl;
  //}
  //for(size_t i=0;i<geoms2.size();i++) 
  //{
  //  bbs2[i]=geoms2[i]->GetAABB();
  //  std::cout << bbs2[i].bmin << std::endl;
  //  std::cout << bbs2[i].bmax << std::endl;
  //}

  //int closest1, closest2;
  //double d = space->settings->DistanceLowerBound(space->world, idrobot, idothers, 0, dInf, &closest1, &closest2);
  //std::cout << std::string(80, '-') << std::endl;
  //std::cout << d << std::endl;
  //std::cout << closest1 << std::endl;
  //std::cout << closest2 << std::endl;
  //
  //exit(0);
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //returns body id of closest
  //space->settings->DistanceLowerBound(space->world, idrobot, idothers, 0, Inf,int* closest1=NULL,int* closest2=NULL);

  return d;
}


//same as Singlerobotcspace but ignore other robots
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


OMPLValidityCheckerInnerOuter::OMPLValidityCheckerInnerOuter(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_, CSpace *outer_):
  OMPLValidityChecker(si, ompl_space_, inner_), outer(outer_)
{
}
bool OMPLValidityCheckerInnerOuter::isValid(const ob::State* state) const
{
  const ob::StateSpacePtr ssp = si_->getStateSpace();
  Config q = ompl_space->OMPLStateToConfig(state);
  SingleRobotCSpace* csi = static_cast<SingleRobotCSpace*>(inner);
  SingleRobotCSpace* cso = static_cast<SingleRobotCSpace*>(outer);
  return IsCollisionFree(csi, q) && (!IsCollisionFree(cso,q));
}

OMPLValidityCheckerNecessarySufficient::OMPLValidityCheckerNecessarySufficient(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *outer_):
  OMPLValidityChecker(si, ompl_space_, ompl_space_->GetCSpacePtr()), outer(outer_)
{
}
bool OMPLValidityCheckerNecessarySufficient::IsSufficient(const ob::State* state) const
{
  Config q = ompl_space->OMPLStateToConfig(state);
  SingleRobotCSpace* cso = static_cast<SingleRobotCSpace*>(outer);
  return IsCollisionFree(cso, q);
}
