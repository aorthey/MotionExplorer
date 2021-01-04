#include "info.h"
#include <KrisLibrary/geometry/AnyGeometry.h>


Info::Info()
{
  //std::cout << std::string(80, '-') << std::endl;
  //std::cout << "Information Module initialized" << std::endl;
  //std::cout << std::string(80, '-') << std::endl;
}

void Info::operator()(const MilestonePath &path)
{
  std::cout <<  path.Length() << std::endl;
}
void Info::operator()(const KinodynamicMilestonePath &path)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "----- KinodynamicMilestonePath Start ------ " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Milestones  : " << path.milestones.size() << std::endl;
  std::cout << "Controls    : " << path.controls.size() << std::endl;
  std::cout << "Paths       : " << path.paths.size() << std::endl;
  //std::cout << "Edges       : " << path.edges.size() << std::endl;
  //std::cout << path.edges.at(0) << std::endl;
  //std::cout << path.edges.at(0)->Space() << std::endl;
  //std::cout <<  path.PathLength() << std::endl;
    // std::vector<State> milestones;
    //   std::vector<ControlInput> controls;
    //     std::vector<std::vector<State> > paths;
    //       std::vector<SmartPointer<EdgePlanner> > edges;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "----- KinodynamicMilestonePath End ------ " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << std::endl;


}
void Info::operator()(const MultiPath &path)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "----- MultiPath Start ------ " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Time start    : " << path.StartTime() << std::endl;
  std::cout << "Time end      : " << path.EndTime() << std::endl;
  std::cout << "Path duration : "<< path.Duration() << std::endl;
  std::cout << "Path valid    : "<< ((path.IsValid())?("True"):("False"))<< std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "----- MultiPath End ------ " << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

void Info::operator()(Robot *robot){
  std::cout << "*** " << std::string(80, '-') << std::endl;
  std::cout << "*** " << " Robot " << robot->name << std::endl;
  std::cout << "*** " << std::string(80, '-') << std::endl;

  vector<string> linkNames = robot->linkNames;
  vector<RobotLink3D> links = robot->links;
  assert( links.size() == linkNames.size() );
  std::cout << "#Links: " << links.size() << std::endl;
  for(uint i = 0; i < links.size(); i++){
    std::cout<< "Link[" << i << "] " << linkNames[i] << " mass " << links[i].mass << std::endl;
  }

  std::cout << std::string(80, '-') << std::endl;
  vector<RobotJoint> joints = robot->joints;
  std::cout << "#Joints: " << joints.size() << std::endl;
  for(uint i = 0; i< joints.size(); i++){
    std::cout<< "Joint[" << i << "] linkidx " << joints[i].linkIndex << " type ";
      switch(joints[i].type){
        case RobotJoint::Floating: std::cout << "floating"; break;
        case RobotJoint::Weld: std::cout << "--"; break;
        case RobotJoint::Normal: std::cout << "actuated"; break;
        case RobotJoint::Spin: std::cout << "infinite rotating"; break;
        default: std::cout << "UNKNOWN"; break;
      }
    std::cout << std::endl;
  }

  std::cout << std::string(80, '-') << std::endl;
  vector<RobotJointDriver> drivers = robot->drivers;
  vector<string> dnames = robot->driverNames;
  std::cout << "#Drivers: " << drivers.size() << std::endl;
  for(uint i = 0; i < drivers.size(); i++){
    double ql = drivers[i].qmin;
    double qu = drivers[i].qmax;
    double vl = drivers[i].vmin;
    double vu = drivers[i].vmax;
    double al = drivers[i].amin;
    double au = drivers[i].amax;
    double tl = drivers[i].tmin;
    double tu = drivers[i].tmax;
    std::cout << "Driver[" << drivers[i].linkIndices.at(0) << "] ";

    std::cout << ql << "<=   q <=" << qu << " | "
              << vl << "<=  dq <=" << vu << " | "
              << al << "<=  ddq <=" << au << " | "
              << tl << "<=  trq <=" << tu << "   ";
    std::cout << "("<< dnames[i] <<")"<<std::endl;
  }

  uint Neffective = 0;
  for(int i = 0; i < robot->qMin.size(); i++){
    double qL = robot->qMin[i];
    double qU = robot->qMax[i];
    if(fabs(qU-qL) > 1e-8) Neffective++;
    //std::cout << "#qlimit [" << i << "] " << robot->qMin[i] << " - " <<  robot->qMax[i] << std::endl;
  }
  //SE(3) element: X Y Z yaw pitch roll
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "#config q=" << robot->q << std::endl;
  std::cout << "#X Y Z          : " << robot->q[0] << ","  << robot->q[1] << "," << robot->q[2]<< std::endl;
  std::cout << "#Yaw Pitch Roll : " << robot->q[3] << ","  << robot->q[4] << "," << robot->q[5]<< std::endl;
  std::cout << "#qmin " << robot->qMin << std::endl;
  std::cout << "#qmax " << robot->qMax << std::endl;
  std::cout << "Dimensionality          : " << robot->qMin.size() << std::endl;
  std::cout << "Effective dimensionality: " << Neffective << " (removing zero measure dimensions)" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  for(int i = 0; i < robot->qMin.size(); i++){
    double qL = robot->qMin[i];
    double qU = robot->qMax[i];
    double vL = robot->velMin[i];
    double vU = robot->velMax[i];
    double aL = -robot->accMax[i];
    double aU = robot->accMax[i];
    if(fabs(qU-qL) > 1e-8) 
    std::cout << "#q= [" << i << "] " << qL << "<=   q <=" << qU << " | "
                                      << vL << "<=  dq <=" << vU << " | "
                                      << aL << "<= ddq <=" << aU << std::endl;
  }
  std::cout << "Current Config Invalidations" << std::endl;
  for(int i = 0; i < robot->qMin.size(); i++){
    double qL = robot->qMin[i];
    double qU = robot->qMax[i];
    double q = robot->q[i];
    if(q < qL || q > qU){
    std::cout << "ERROR: #q= [" << i << "] " << qL << "<= " << q << " <=" << qU << std::endl;
    }
  }
  if(robot->SelfCollision()){
    for(int j = 0; j < robot->selfCollisions.numRows(); j++){
      for(int k = j+1; k < robot->selfCollisions.numCols(); k++){
        AnyCollisionQuery* cq = robot->selfCollisions(j,k);
        if(cq && cq->Collide()){
          std::cout << "ERROR (Self Collision): " << linkNames.at(j) << " <-> " << linkNames.at(k) << std::endl;
        }
      }
    }
  }
  if(robot->MeshCollision(0))
  {
    for(uint j = 0; j < robot->envCollisions.size(); j++)
    {
      AnyCollisionQuery* cq = robot->envCollisions.at(j);
      if(cq && cq->Collide())
      {
        std::cout << "ERROR (Env Collision): " << std::endl;
      }
    }
  }

}

void Info::operator()(RobotWorld *world){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "RobotWorld Info" << std::endl;
  std::cout << std::string(80, '-') << std::endl;

  int ids = world->NumIDs();

  std::cout << std::string(80, '-') << std::endl;
  for(int itr = 0; itr <= ids; itr++){
    std::cout << "[" << itr << "] " << world->GetName(itr) << std::endl;
  }

  std::vector<SmartPointer<Robot> > robots = world->robots;
  std::vector<SmartPointer<Terrain> > terrains = world->terrains;
  for (std::vector<SmartPointer<Robot> >::iterator it = robots.begin() ; it != robots.end(); ++it){
    (*this)(*it);
  }//for all robots

}
void Info::operator()(WorldSimulation *sim){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "WorldSimulation Info" << std::endl;
  std::cout << std::string(80, '-') << std::endl;


  for(uint k = 0; k < sim->controlSimulators.size(); k++){
    const ControlledRobotSimulator rs = sim->controlSimulators.at(k);
    std::cout << "--- robot [" << rs.robot->name << "] controller " << rs.controller->Type()<< std::endl;
    std::vector<string> cmds = rs.controller->Commands();
    for(uint j = 0; j < cmds.size(); j++){
      std::cout << "    cmd[" << j << "] " << cmds.at(j) << std::endl;
    }
  }
  std::cout << "Available Controller" << std::endl;
  for(uint k = 0; k < sim->robotControllers.size(); k++){
    SmartPointer<RobotController> rs = sim->robotControllers.at(k);
    std::cout << " controller " << k << ": " << rs->Type()<< std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
}

void Info::operator()(const Terrain *terrain)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Terrain Mesh Info" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  const CollisionMesh mesh = terrain->geometry->TriangleMeshCollisionData();
  std::cout << "Number Triangles : " << mesh.tris.size() << std::endl;
  for(uint j = 0; j < mesh.tris.size(); j++){
    Triangle3D tri;
    mesh.GetTriangle(j, tri);
    std::cout << "Triangle " << j << std::endl;
    std::cout << "   normal   : " << tri.normal() << std::endl;
    std::cout << "   vertex1  : " << tri.vertex(0) << std::endl;
    std::cout << "   vertex2  : " << tri.vertex(1) << std::endl;
    std::cout << "   vertex3  : " << tri.vertex(2) << std::endl;
    std::cout << "   area     : " << tri.area() << std::endl;
    Vector3 c;
    std::cout << tri.barycentricCoords(c) << std::endl;


  }
}
