#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "common.h"
#include "gui/colors.h"
#include <ompl/base/StateSpaceTypes.h>
#include <KrisLibrary/GLdraw/drawextra.h>

std::vector<ManagedGeometry::GeometryPtr> 
getEnvironmentCollisionGeometries(SingleRobotCSpace *space)
{
  std::vector<ManagedGeometry::GeometryPtr> collisionGeometries;
  for(uint k = 0; k < space->world.rigidObjects.size(); k++)
  {
      const ManagedGeometry::GeometryPtr tgeom = 
        space->world.rigidObjects[k]->geometry;
      collisionGeometries.push_back(tgeom);
  }
  for(uint k = 0; k < space->world.terrains.size(); k++)
  {
      const ManagedGeometry::GeometryPtr tgeom = 
        space->world.terrains[k]->geometry;
      collisionGeometries.push_back(tgeom);
  }
  return collisionGeometries;
}


OMPLValidityChecker::OMPLValidityChecker(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_):
  ob::StateValidityCheckerDifferentiable(si), cspace(cspace_)
{
    klampt_single_robot_cspace = static_cast<SingleRobotCSpace*>(cspace_->GetCSpaceKlamptPtr());
    specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::BOUNDED_APPROXIMATE;
    tmp = si_->allocState();
}

OMPLValidityChecker::~OMPLValidityChecker()
{
    si_->freeState(tmp);
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
  std::lock_guard<std::recursive_mutex> guard(cspace->getLock());

  Config q = cspace->OMPLStateToConfig(state);
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
  return c;
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
  if(res.first >= 0)
  {
    return false;
  }
  //environment collision checking
  res = space->settings->CheckCollision(space->world,idrobot,idothers);

  if(res.first >= 0)
  {
    // std::vector<std::pair<int,int>> pairs;
    // std::vector<Geometry::AnyCollisionQuery> queries;

    // space->settings->EnumerateCollisionQueries(space->world, -1, -1, pairs, queries);

    // for(uint k = 0; k < queries.size(); k++){
    //   std::cout << queries.at(k).PenetrationDepth() << std::endl;
    // }
    // std::cout << "env-collision: " << res.first << "<->" << res.second << std::endl;
    return false;
  }

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


double OMPLValidityChecker::cost(const ob::State *state) const
{
    double distToConstraints = DistanceToRobot(state, klampt_single_robot_cspace);
    //model it

    if(distToConstraints <= 0)
    {
      return dInf;
    }else{
      return -(distToConstraints*distToConstraints);
    }
}

double OMPLValidityChecker::constrainedness(const ob::State *state) const
{
  //update robot geometry to state
  Config q = cspace->OMPLStateToConfig(state);
  SingleRobotCSpace *space = klampt_single_robot_cspace;
  Robot* robot = space->GetRobot();
  robot->UpdateConfig(q);
  robot->UpdateGeometry();

  std::vector<ManagedGeometry::GeometryPtr> collisionGeometries 
    = getEnvironmentCollisionGeometries(space);

  uint N = si_->getStateDimension();
  Vector torqueTotal(N);
  torqueTotal.setZero();

  InteractionPoints interactionPoints = getInteractionPoints(robot, collisionGeometries);
  // typedef std::vector<std::pair<Vector3, Vector3> > InteractionPoints;

  //constrainedness depends on:
  //(1) distance to constraint
  //(2) number of opposing constraints
  for(uint k = 0; k < collisionGeometries.size(); k++)
  {
    const ManagedGeometry::GeometryPtr tgeom = collisionGeometries.at(k);
    Vector wrench_k = getVirtualForceRobotToMesh(robot, tgeom);
  }

  Eigen::VectorXd dx(si_->getStateDimension());
  for(uint k = 0; k < si_->getStateDimension(); k++)
  {
    dx(k) = torqueTotal[k];
  }

  return dx.norm();

}

void adjustForce(Vector3 &f)
{
  double eta = 0.01; //gain on repulsive gradient
  double dObstacleStar = 0.5; //ignore obstacles w distance above dObstacleStar

  double d = f.norm();

  if(d <= dObstacleStar)
  {
      f = eta*(-1.0/dObstacleStar + 1.0/d)*(1.0/(d*d))*f;
  }else
  {
      f.setZero();
  }
}

Vector OMPLValidityChecker::getVirtualForceRobotToMesh(
    Robot *robot,
    const ManagedGeometry::GeometryPtr geometry
    ) const
{
  Geometry::AnyCollisionGeometry3D tt(*geometry);
  robot->CleanupCollisions();
  robot->InitMeshCollision(tt);

  uint N = si_->getStateDimension();
  Vector wrenches(N);
  wrenches.setZero();

  for(uint i = 0; i < robot->links.size(); i++)
  {
      if(robot->IsGeometryEmpty(i)) continue;

      RobotLink3D *link = &robot->links[i];

      Geometry::AnyCollisionQuery *query = robot->envCollisions[i];
      std::vector<Vector3> vp1,vp2;

      query->Distance(0,0);
      query->InteractingPoints(vp1,vp2);

      if(vp1.size() >= 1)
      {
        //p1 point on robot
        //p2 point on obstacle
        Vector3 p1 = link->T_World*vp1.front();
        Vector3 p2 = vp2.front();
        p2 = geometry->currentTransform*p2;

        Vector3 Force_workspace = p1-p2;

        adjustForce(Force_workspace);

        if(Force_workspace.norm() > 0)
        {
          ///for a force f at pi on link i, returns joint wrenches F = J^t f
          Vector wrenchesAtLink;
          robot->GetForceTorques(Force_workspace, vp1.front(), i, wrenchesAtLink);

          cspace->ConfigToOMPLState(wrenchesAtLink, tmp);

          std::vector<double> wrenchVec;
          si_->getStateSpace()->copyToReals(wrenchVec, tmp);

          wrenches += wrenchVec;
        }
      }
  }
  return wrenches;
}


OMPLValidityChecker::InteractionPoints OMPLValidityChecker::getInteractionPoints(
    Robot *robot,
    std::vector<ManagedGeometry::GeometryPtr> collisionGeometries 
    ) const
{

  InteractionPoints interactionPoints;

  // std::cout << "Checking against " << collisionGeometries.size() 
  //   << " collisionGeometries."  << std::endl;
  // std::cout << "Config: " << robot->q << std::endl;
  for(uint k = 0; k < collisionGeometries.size(); k++)
  {
      const ManagedGeometry::GeometryPtr tgeom = collisionGeometries.at(k);
      Geometry::AnyCollisionGeometry3D tt(*tgeom);
      robot->CleanupCollisions();
      robot->InitMeshCollision(tt);

      for(uint i = 0; i < robot->links.size(); i++)
      {
          if(robot->IsGeometryEmpty(i)) continue;

          RobotLink3D *link = &robot->links[i];

          Geometry::AnyCollisionQuery *query = robot->envCollisions[i];
          std::vector<Vector3> vp1,vp2;

          query->Distance(0,0);
          query->InteractingPoints(vp1,vp2);

          if(vp1.size() >= 1)
          {
            Vector3 p1 = link->T_World * vp1.front();
            Vector3 p2 = tgeom->currentTransform * vp2.front();

            interactionPoints.push_back(std::make_pair(p1, p2));
            // std::cout << "Interaction " << i <<":" << p1 << "," << p2 << std::endl;
          }
      }
  }
  return interactionPoints;
}

Eigen::VectorXd OMPLValidityChecker::costGradient(const ob::State *state) const
{
  //update robot geometry to state
  Config q = cspace->OMPLStateToConfig(state);

  SingleRobotCSpace *space = klampt_single_robot_cspace;
  Robot* robot = space->GetRobot();
  robot->UpdateConfig(q);
  robot->UpdateGeometry();

  std::vector<ManagedGeometry::GeometryPtr> collisionGeometries 
    = getEnvironmentCollisionGeometries(space);

  uint N = si_->getStateDimension();
  Vector torqueTotal(N);
  torqueTotal.setZero();
  for(uint k = 0; k < collisionGeometries.size(); k++)
  {
      const ManagedGeometry::GeometryPtr tgeom = collisionGeometries.at(k);
      torqueTotal+= getVirtualForceRobotToMesh(robot, tgeom);
  }

  Eigen::VectorXd dx(si_->getStateDimension());
  for(uint k = 0; k < si_->getStateDimension(); k++)
  {
    dx(k) = torqueTotal[k];
  }

  // std::cout << "Force " << dx << std::endl;

  return dx;
}

void OMPLValidityChecker::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glEnable(GL_LINE_SMOOTH);
  glDisable(GL_CULL_FACE);
  SingleRobotCSpace *space = klampt_single_robot_cspace;
  Robot* robot = space->GetRobot();

  std::vector<ManagedGeometry::GeometryPtr> collisionGeometries 
    = getEnvironmentCollisionGeometries(space);

  InteractionPoints interactionPoints = getInteractionPoints(robot, collisionGeometries);
  //assume robot has been updated
  // std::cout << "Found " << interactionPoints.size() << " interaction pairs." << std::endl;
  GLColor yellow(1,1,0);
  setColor(yellow);
  for(uint k = 0; k < interactionPoints.size(); k++)
  {
    Vector3 p1 = interactionPoints.at(k).first;
    Vector3 p2 = interactionPoints.at(k).second;
    glPushMatrix();
    glLineWidth(3);
    glPointSize(5);
    GLDraw::drawPoint(p1);
    GLDraw::drawPoint(p2);
    glBegin(GL_LINES);
    glVertex3f(p1[0],p1[1],p1[2]);
    glVertex3f(p2[0],p2[1],p2[2]);
    glEnd();
    glPopMatrix();
  }
  glEnable(GL_CULL_FACE);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}
