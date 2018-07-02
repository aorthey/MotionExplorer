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
  std::cout << N << std::endl;
  std::cout << A << std::endl;
  std::cout << b << std::endl;
  problem.setBounds(iris::Polyhedron(A,b));

  Eigen::MatrixXd obs(3,3);

  for(uint k = 0; k < world->terrains.size(); k++)
  {
    SmartPointer<Terrain> terrain = world->terrains.at(k);
    ManagedGeometry& geometry = terrain->geometry;
    const Meshing::TriMesh& trimesh = geometry->AsTriangleMesh();
    uint N = trimesh.tris.size();
    //Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(N,N);
    //Eigen::MatrixXd D = Eigen::MatrixXd::Zero(N,N);
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

      // int tri0 = trimesh.GetAdjacentTri(i, 0);
      // int tri1 = trimesh.GetAdjacentTri(i, 1);
      // int tri2 = trimesh.GetAdjacentTri(i, 2);

      // if(tri0>=0) Adj(i, tri0) = 1;
      // if(tri1>=0) Adj(i, tri1) = 1;
      // if(tri2>=0) Adj(i, tri2) = 1;
      // D(i,i) = Adj.row(i).sum();

    }
    // Eigen::MatrixXd L = D - Adj; //laplacian of triangle mesh
    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(L);
    // Eigen::MatrixXd ker = lu_decomp.kernel();
    // std::cout << "Connected Components in Trimesh " << k << " : " << ker.cols() << std::endl;
  }

  std::cout << "seed_pt: " << seed_pt << std::endl;
  problem.setSeedPoint(seed_pt);
  iris::IRISOptions options;
  options.require_containment = true;
  iris::IRISDebugData debug;
  //iris::IRISRegion region = inflate_region(problem, options, &debug);
  iris::IRISRegion region = inflate_region(problem, options);

  double d = region.getEllipsoid().getVolume();
  if(d>=1e-5){
    cover::OpenSetConvex *cvx_region = new cover::OpenSetConvex(cspace, state, region, problem.getBounds());
    return cvx_region;
  }else{
    return nullptr;
  }
}

std::vector<cover::OpenSetConvex*> ValidityCheckerSimplicialComplex::GetConvexWorkspaceCover(const ob::State* start, const ob::State* goal) const
{
  const ob::RealVectorBounds& bounds = static_pointer_cast<ob::RealVectorStateSpace>(cspace->SpacePtr())->getBounds();
  uint N = cspace->GetDimensionality();
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

  ConvexPolyhedron *polyhedron_bounds = new ConvexPolyhedron(A, b);
  NefPolyhedron *nef_polyhedron = new NefPolyhedron( polyhedron_bounds );
  nef_polyhedron->SubtractObstacles(cspace);

  std::vector<ConvexPolyhedron> cvx_decomposition = nef_polyhedron->GetConvexDecomposition();

  std::vector<cover::OpenSetConvex*> open_sets_cvx;
  for(uint k = 0; k < cvx_decomposition.size(); k++){
    ConvexPolyhedron& cp = cvx_decomposition.at(k);
    Eigen::VectorXd c = cp.GetGeometricCenter();
    Config q(3);
    for(uint j = 0; j < 3; j++) q(j) = c(j);
    ob::State *s = si_->allocState();
    cspace->ConfigToOMPLState(q, s);

    cover::OpenSetConvex *osc = new cover::OpenSetConvex(cspace, s, &cp);
    open_sets_cvx.push_back(osc);
  }
  return open_sets_cvx;
}
