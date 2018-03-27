#include "validity_checker_simplicial_complex.h"
#include "common.h"
#include <KrisLibrary/meshing/TriMesh.h>
#include <iostream>
#include <Eigen/Core>
#include <iris/iris.h>

void testIRIS(){
  iris::IRISProblem problem(3);
  // Inflate a region inside a 1x1 box
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

ValidityCheckerSimplicialComplex::ValidityCheckerSimplicialComplex(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_):
  OMPLValidityChecker(si, ompl_space_, inner_)
{
}

cover::OpenSetConvex ValidityCheckerSimplicialComplex::ComputeNeighborhood(const ob::State* state) const
{
  if(!isValid(state)){
    std::cout << "need valid state" << std::endl;
    return cover::OpenSetConvex();
  }

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

  uint N = ompl_space->GetDimensionality();

  iris::IRISProblem problem(N);

  Eigen::VectorXd seed_pt(N);
  for(uint k = 0; k < N; k++){
    seed_pt(k) = q(k); //@TODO: not always true
  }


  const ob::RealVectorBounds& bounds = static_pointer_cast<ob::RealVectorStateSpace>(ompl_space->SpacePtr())->getBounds();
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
    //std::cout << "terrain: " << terrain->name << std::endl;
    ManagedGeometry& geometry = terrain->geometry;
    //std::cout << geometry->TypeName() << std::endl;
    const Meshing::TriMesh& trimesh = geometry->AsTriangleMesh();
    //std::cout << "triangles: " << trimesh.tris.size() << std::endl;
    for(uint i = 0; i < trimesh.tris.size(); i++){
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
  iris::IRISRegion region = inflate_region(problem, options);

  cover::OpenSetConvex cvx_region(region);

  return cvx_region;

  //pair<int,int> res;
  //selfcollision checking
  // res = space->settings->CheckCollision(space->world,idrobot);
  // if(res.first >= 0) return 0;
  // //environment collision checking
  // res = space->settings->CheckCollision(space->world,idrobot,idothers);
  // if(res.first >= 0) return 0;
}


