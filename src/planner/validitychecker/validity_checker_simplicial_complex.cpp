#include "validity_checker_simplicial_complex.h"
#include "common.h"
#include <KrisLibrary/meshing/TriMesh.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iris/iris.h>

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
    uint N = trimesh.tris.size();
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(N,N);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(N,N);
    for(uint i = 0; i < N; i++)
    {
      Vector3 v0 = trimesh.TriangleVertex(i,0);
      Vector3 v1 = trimesh.TriangleVertex(i,1);
      Vector3 v2 = trimesh.TriangleVertex(i,2);
      //std::cout << "tri" << i << " : " << v0 << v1 << v2 << std::endl;
      obs << v0[0], v1[0], v2[0],
             v0[1], v1[1], v2[1],
             v0[2], v1[2], v2[2];
      problem.addObstacle(obs);

      //Vector3 ni = trimesh.TriangleNormal(i);
      int tri0 = trimesh.GetAdjacentTri(i, 0);
      int tri1 = trimesh.GetAdjacentTri(i, 1);
      int tri2 = trimesh.GetAdjacentTri(i, 2);

      if(tri0>=0) Adj(i, tri0) = 1;
      if(tri1>=0) Adj(i, tri1) = 1;
      if(tri2>=0) Adj(i, tri2) = 1;
      D(i,i) = Adj.row(i).sum();

    }
    std::cout << Adj << std::endl;
    Eigen::MatrixXd L = D - Adj; //laplacian of triangle mesh
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(L);
    Eigen::MatrixXd ker = lu_decomp.kernel();
    std::cout << "Connected Components in Trimesh " << k << " : " << ker.cols() << std::endl;
  }

  problem.setSeedPoint(seed_pt);

  iris::IRISOptions options;
  options.require_containment = true;
  iris::IRISDebugData debug;
  iris::IRISRegion region = inflate_region(problem, options, &debug);

  //uint M = region.polyhedron.getNumberOfConstraints();
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
