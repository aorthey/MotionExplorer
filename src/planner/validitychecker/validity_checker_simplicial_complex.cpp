#include "validity_checker_simplicial_complex.h"
#include "common.h"
#include <KrisLibrary/meshing/TriMesh.h>
#include <iostream>
#include <Eigen/Core>
#include <iris/iris.h>

void testIRIS(){
  iris::IRISProblem problem(3);
  // Inflate a region inside a 1x1x1 box
  problem.setSeedPoint(Eigen::Vector3d(0.1, 0.1, 0.1));

  //obstacle is defined by Matrix NxM whereby N is the number of dimensions, and
  //M are the number of points. It is assumed that the obstacle is convex, i.e.
  //the obstacle is defined as the convex hull of all the given points
  Eigen::MatrixXd obs(3,4);
  //left
  obs << 0, 0, 0, 0,
         0, 1, 1, 0,
         0, 1, 0, 1;
  problem.addObstacle(obs);
  //right
  obs << 1, 1, 1, 1,
         0, 1, 1, 0,
         0, 1, 0, 1;
  problem.addObstacle(obs);

  //front
  obs << 0, 1, 1, 0,
         0, 0, 0, 0,
         0, 0, 1, 1;
  problem.addObstacle(obs);
  //back
  obs << 0, 1, 1, 0,
         1, 1, 1, 1,
         0, 0, 1, 1;
  problem.addObstacle(obs);

  //bottom
  obs << 0, 1, 0, 1,
         0, 0, 1, 1,
         0, 0, 0, 0;
  problem.addObstacle(obs);
  //top
  obs << 0, 1, 0, 1,
         0, 0, 1, 1,
         1, 1, 1, 1;
  problem.addObstacle(obs);

  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);

  std::cout << "C: " << region.ellipsoid.getC() << std::endl;
  std::cout << "d: " << region.ellipsoid.getD() << std::endl;
  std::cout << "A: " << region.polyhedron.getA() << std::endl;
  std::cout << "b: " << region.polyhedron.getB() << std::endl;

}

ValidityCheckerSimplicialComplex::ValidityCheckerSimplicialComplex(const ob::SpaceInformationPtr &si, CSpaceOMPL *cspace_, CSpace *inner_):
  OMPLValidityChecker(si, cspace_, inner_)
{
}

cover::OpenSetConvex* ValidityCheckerSimplicialComplex::ComputeNeighborhood(const ob::State* state) const
{
  if(!isValid(state)){
    std::cout << "need valid state" << std::endl;
    return nullptr;
  }

  Config q = cspace->OMPLStateToConfig(state);
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

  uint N = cspace->GetDimensionality();

  iris::IRISProblem problem(3);

  Eigen::VectorXd seed_pt(3);
  Vector3 qp = cspace->getXYZ(state);
  for(uint k = 0; k < 3; k++){
    seed_pt[k] = qp[k];
  }

  const ob::RealVectorBounds& bounds = static_pointer_cast<ob::RealVectorStateSpace>(cspace->SpacePtr())->getBounds();
  // std::cout << bounds.low << std::endl;
  // std::cout << bounds.high << std::endl;

  Eigen::MatrixXd A(2*N,N);
  for(uint k = 0; k < N; k++){
    for(uint j = 0; j < N; j++){
      A(k,  j) = (k==j?-1:0);
      A(k+N,j) = (k==j?+1:0);
    }
  }
  Eigen::VectorXd b(2*N);
  for(uint k = 0; k < N; k++){
    b(k)   = -bounds.low[k];
    b(k+N) = bounds.high[k];
  }
  problem.setBounds(iris::Polyhedron(A,b));

  Eigen::MatrixXd obs(3,3);
  for(uint k = 0; k < world->terrains.size(); k++)
  {
    SmartPointer<Terrain> terrain = world->terrains.at(k);
    ManagedGeometry& geometry = terrain->geometry;
    const Meshing::TriMesh& trimesh = geometry->AsTriangleMesh();
    for(uint i = 0; i < trimesh.tris.size(); i++)
    {
      Vector3 v0 = trimesh.verts.at( trimesh.tris.at(i)[0] );
      Vector3 v1 = trimesh.verts.at( trimesh.tris.at(i)[1] );
      Vector3 v2 = trimesh.verts.at( trimesh.tris.at(i)[2] );
      //std::cout << "tri" << i << " : " << v0 << v1 << v2 << std::endl;
      obs << v0[0], v1[0], v2[0],
             v0[1], v1[1], v2[1],
             v0[2], v1[2], v2[2];
      problem.addObstacle(obs);
    }
  }
  problem.setSeedPoint(seed_pt);

  iris::IRISOptions options;
  options.require_containment = true;
  iris::IRISDebugData debug;
  iris::IRISRegion region = inflate_region(problem, options, &debug);
  uint M = region.polyhedron.getNumberOfConstraints();

  std::cout << M << std::endl;

  // std::vector<iris::Polyhedron> phistory = debug.polyhedron_history;
  // std::vector<iris::Ellipsoid> ehistory = debug.ellipsoid_history;

  // for(uint k = 0; k < phistory.size(); k++){
  //   std::cout << std::string(80, '-') << std::endl;
  //   std::cout << "history: " << k << "/" << phistory.size() << " constraints: " << phistory.at(k).getNumberOfConstraints() << std::endl;
  //   std::cout << phistory.at(k).getA() << std::endl;
  //   std::cout << phistory.at(k).getB() << std::endl;
  //   bool contains_sp = phistory.at(k).contains(seed_pt,1e-10);
  //   std::cout << "seed point: " << seed_pt << std::endl;
  //   std::cout << (contains_sp?"OK":"violation") << std::endl;
  //   if(!contains_sp){
  //     exit(0);
  //   }
  // }

  cover::OpenSetConvex *cvx_region = new cover::OpenSetConvex(cspace, state, region);
  return cvx_region;
}
