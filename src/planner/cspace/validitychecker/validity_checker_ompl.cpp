#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include "common.h"
#include "gui/colors.h"
#include <ompl/base/StateSpaceTypes.h>
#include <KrisLibrary/GLdraw/drawextra.h>

//convert t in [t0, t1] (default: [0.0, 1.0]) to a value in [x0, x1] (default:
//[1.0, 0.0]), whereby values of t closer to t0 are assigned values closer to x0
//and values closer to t1 are assigned values closer to x1 (smoothly
//interpolated)

double stepfunction(double t, double t0=0.0, double t1=1.0, double x0=1.0, double x1=0.0)
{
  double x = (t-t0)/(t1-t0);

  //catch out of bound values
  if(x < 0.0) return x0;
  if(x >= 1.0) return x1;

  //smooth segment using hermite polynomial
  return x0 + (x1-x0)*(x*x*(3-2*x));
}

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
      // std::cout << tgeom->TypeName() << std::endl;
        //const Meshing::TriMesh& AsTriangleMesh() const;

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
          double d = query->Distance(0,0);

          std::vector<Vector3> vp1,vp2;
          query->InteractingPoints(vp1,vp2);

          if(vp1.size() >= 1)
          {
            InteractionPoint ip;
            ip.ptRobot = link->T_World * vp1.front();
            ip.ptEnvironment = tgeom->currentTransform * vp2.front();
            ip.distance = d;
            ip.robot = robot;
            ip.link = i;
            ip.collisionGeometriesId = k;

            ///for a force f at pi on link i, returns joint wrenches F = J^t f
            Vector3 f = ip.ptRobot - ip.ptEnvironment;

            Vector3 l(0,0,1);

            double d = stepfunction(f.normSquared(), 0, 10, 1.0, 0.1);
            int q = (k%2)*2 - 1;

            Vector3 b = q*cross(l,f)/d;

            Vector wrenchesAtLink;
            //requires the local point coordinate
            robot->GetForceTorques(b, vp1.front(), ip.link, wrenchesAtLink);

            // std::cout << "Force from geometry " << k << " : " << b << std::endl;

            ip.wrench = wrenchesAtLink;

            // std::cout << "Wrench from geometry " << k << " : " << ip.wrench << std::endl;
            // cspace->ConfigToOMPLState(wrenchesAtLink, tmp);

            // std::vector<double> wrenchVec;
            // si_->getStateSpace()->copyToReals(wrenchVec, tmp);

            // wrenches += wrenchVec;
            interactionPoints.push_back(ip);
          }
      }
  }
  return interactionPoints;
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
    Vector3 p1 = interactionPoints.at(k).ptRobot;
    Vector3 p2 = interactionPoints.at(k).ptEnvironment;
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

struct interactionPointCompare
{
  inline bool operator() (const InteractionPoint& ip1, const InteractionPoint& ip2)
  {
    return (ip1.distance < ip2.distance);
  }
};

double costFromInteractionPoints(InteractionPoint& ip1, InteractionPoint& ip2)
{
    ip1.distance = ip1.wrench.norm();
    ip2.distance = ip2.wrench.norm();

    ip1.wrench /= ip1.distance;
    ip2.wrench /= ip2.distance;

    //-1.0 to 1.0
    double d = dot(ip1.wrench, ip2.wrench);
    // double pinchness = std::max(stepfunction(d, -1.0, +0.0),
    //     stepfunction(d,1.0,0.0));
    double pinchness = stepfunction(d, -1.0, +0.0);

    //NOTES: 
    //-- long tails == bad (because they stretch too much in high-dim spaces
    //which would confuse sampler)
    //-- crisp tight ridges is what we want (easier to travel along, but curves
    //should be smooth)
    //-- stay out of corners (might not be totally possible)

    double equalness = stepfunction(fabs(ip1.distance - ip2.distance), 0.0, 2.0);

    double lowest_distance = std::min(ip1.distance, ip2.distance);
    double strength = stepfunction(lowest_distance, 0.0, 2.0);

    double constrainedness = equalness * strength * pinchness;
    // std::cout << "Best distance: " << ip1.distance << "," << ip2.distance << " :"<< 
    //   constrainedness << std::endl;
    return constrainedness;
}

double costFromInteractionPointsMagnetic(InteractionPoint& ip1, InteractionPoint& ip2)
{
    ip1.distance = ip1.wrench.norm();
    ip2.distance = ip2.wrench.norm();

    auto ip_mid = 0.5*(ip1.wrench + ip2.wrench);

    return ip_mid.norm();
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

  std::sort(interactionPoints.begin(), interactionPoints.end(), interactionPointCompare());

  if(interactionPoints.size() < 2)
  {
    std::cout << "No interaction points (" << interactionPoints.size() << ")" << std::endl;
    return 0;
  }else{
    InteractionPoint ip1 = interactionPoints.at(0);
    InteractionPoint ip2 = interactionPoints.at(1);
    return costFromInteractionPointsMagnetic(ip1, ip2);
  }

}

//Eigen::VectorXd OMPLValidityChecker::costGradient(const ob::State *state) const
//{
//  //update robot geometry to state
//  Config q = cspace->OMPLStateToConfig(state);

//  SingleRobotCSpace *space = klampt_single_robot_cspace;
//  Robot* robot = space->GetRobot();
//  robot->UpdateConfig(q);
//  robot->UpdateGeometry();

//  std::vector<ManagedGeometry::GeometryPtr> collisionGeometries 
//    = getEnvironmentCollisionGeometries(space);

//  uint N = si_->getStateDimension();
//  Vector torqueTotal(N);
//  torqueTotal.setZero();
//  for(uint k = 0; k < collisionGeometries.size(); k++)
//  {
//      const ManagedGeometry::GeometryPtr tgeom = collisionGeometries.at(k);
//      torqueTotal+= getVirtualForceRobotToMesh(robot, tgeom);
//  }

//  Eigen::VectorXd dx(si_->getStateDimension());
//  for(uint k = 0; k < si_->getStateDimension(); k++)
//  {
//    dx(k) = torqueTotal[k];
//  }

//  // std::cout << "Force " << dx << std::endl;

//  return dx;
//}

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

  InteractionPoints interactionPoints = getInteractionPoints(robot, collisionGeometries);

  std::sort(interactionPoints.begin(), interactionPoints.end(), interactionPointCompare());

  Eigen::VectorXd dx(si_->getStateDimension());
  dx.setZero();

  if(interactionPoints.size() < 2)
  {
    std::cout << "No interaction points (" << interactionPoints.size() << ")" << std::endl;
  }else{
    InteractionPoint ip1 = interactionPoints.at(0);
    InteractionPoint ip2 = interactionPoints.at(1);

    for(uint k = 0; k < si_->getStateDimension(); k++)
    {
      dx(k) = 0.5*(ip1.wrench[k]+ip2.wrench[k]);
    }
  }

  return dx;

}
