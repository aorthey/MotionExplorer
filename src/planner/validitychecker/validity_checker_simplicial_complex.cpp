#include "validity_checker_simplicial_complex.h"
#include <KrisLibrary/meshing/TriMesh.h>
#include <iostream>
#include <Eigen/Core>
#include <iris/iris.h>


ValidityCheckerSimplicialComplex::ValidityCheckerSimplicialComplex(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_):
  OMPLValidityChecker(si, ompl_space_, inner_)
{
}

void ValidityCheckerSimplicialComplex::ComputeNeighborhood(const ob::State* state) const
{
  if(!isValid(state)) return;

  Config q = ompl_space->OMPLStateToConfig(state);
  SingleRobotCSpace* space = static_cast<SingleRobotCSpace*>(inner);
  Robot* robot = space->GetRobot();
  RobotWorld* world = &space->world;

  robot->UpdateConfig(q);
  robot->UpdateGeometry();

  int id = world->RobotID(space->index);
  vector<int> idrobot(1,id);
  vector<int> idothers;
  for(size_t i=0;i<world->terrains.size();i++)
  {
    idothers.push_back(world->TerrainID(i));
  }
  for(size_t i=0;i<world->rigidObjects.size();i++)
  {
    idothers.push_back(world->RigidObjectID(i));
  }

  // bool CheckCollision(AnyCollisionGeometry3D* m1,AnyCollisionGeometry3D* m2,Real tol)
  // {
  //     if(!m1 || !m2) return false;
  //       Assert(tol >= 0);
  //         AnyCollisionQuery q(*m1,*m2);
  //           if(tol == 0) {
  //                 return q.Collide();
  //                   }
  //             else {
  //                   return q.WithinDistance(tol);
  //                     }
  // }

  // for(uint k = 0; k < robot->geometry.size(); k++){
  //   std::cout << robot->geometry.at(k)->TypeName() << std::endl;
  // }
  for(uint k = 0; k < world->terrains.size(); k++)
  {
    SmartPointer<Terrain> terrain = world->terrains.at(k);
    std::cout << "terrain: " << terrain->name << std::endl;
    ManagedGeometry& geometry = terrain->geometry;
    //typedef SmartPointer<Geometry::AnyCollisionGeometry3D> GeometryPtr;
    //GeometryPtr geometry = terrain->geometry();
    std::cout << geometry->TypeName() << std::endl;
    const Meshing::TriMesh& trimesh = geometry->AsTriangleMesh();
    std::cout << "triangles: " << trimesh.tris.size() << std::endl;
    //typedef IntTriple Tri;
    std::cout << "vertices : " << trimesh.verts.size() << std::endl;
    //triangles are ints(a,b,c) representing vertices which are vector3 in R3
  }


  iris::IRISProblem problem(2);
  problem.setSeedPoint(Eigen::Vector2d(0.1, 0.1));

  Eigen::MatrixXd obs(2,2);
  // Inflate a region inside a 1x1 box
  obs << 0, 1,
         0, 0;
  problem.addObstacle(obs);
  obs << 1, 1,
         0, 1;
  problem.addObstacle(obs);
  obs << 1, 0,
         1, 1;
  problem.addObstacle(obs);
  obs << 0, 0,
         1, 0;
  problem.addObstacle(obs);

  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);

  std::cout << "C: " << region.ellipsoid.getC() << std::endl;
  std::cout << "d: " << region.ellipsoid.getD() << std::endl;

  exit(0);

  //pair<int,int> res;
  //selfcollision checking
  // res = space->settings->CheckCollision(space->world,idrobot);
  // if(res.first >= 0) return 0;
  // //environment collision checking
  // res = space->settings->CheckCollision(space->world,idrobot,idothers);
  // if(res.first >= 0) return 0;
}
