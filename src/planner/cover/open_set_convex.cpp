#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"
#include "3rdparty/Polyhedron.h"
extern "C" {
  #include <qhull/qhull_a.h>
}

// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSetConvex::OpenSetConvex(iris::IRISRegion region_):
  region(region_)
{
}
void OpenSetConvex::DrawGL(GUIState&){
  //Visualize
  // std::cout << "C: " << region.ellipsoid.getC() << std::endl;
  // std::cout << "d: " << region.ellipsoid.getD() << std::endl;
  // std::cout << "A: " << region.polyhedron.getA() << std::endl;
  // std::cout << "b: " << region.polyhedron.getB() << std::endl;

  Eigen::MatrixXd A_eigen = region.polyhedron.getA();
  Eigen::VectorXd b_eigen = region.polyhedron.getB();

  Eigen::MatrixXd C_eigen = region.ellipsoid.getC();
  Eigen::VectorXd d_eigen = region.ellipsoid.getD();

  Vector3 center(d_eigen[0],d_eigen[1],d_eigen[2]);
  Vector3 u(C_eigen(0,0),C_eigen(1,0),C_eigen(2,0));
  Vector3 v(C_eigen(0,1),C_eigen(1,1),C_eigen(2,1));
  Vector3 w(C_eigen(0,2),C_eigen(1,2),C_eigen(2,2));


  GLColor blue(0.1,0.1,0.9,1);
  GLColor grey(0.7,0.7,0.7,1);
  setColor(blue);
  GLDraw::drawWireEllipsoid(center, u, v, w);
  setColor(grey);

  Eigen::Polyhedron P;
  P.vrep(A_eigen, b_eigen);
    /* Get the V-representation of the polyhedron
     * V-polyhedron is such that \f$ A = [v^T r^T]^T, b=[1^T 0^T]^T \f$
     * with A composed of \f$ v \f$, the vertices, \f$ r \f$, the rays
     * and b is a vector which is 1 for vertices and 0 for rays.
     * \return Pair of vertices and rays matrix and identification vector of vertices and rays for the V-representation
     */
    //std::pair<Eigen::MatrixXd, Eigen::VectorXd> vrep() const;
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> VV;
  VV = P.vrep();

  Eigen::MatrixXd V = VV.first;

  std::cout << V.rows() << std::endl;
  std::cout << V.cols() << std::endl;
  for(uint k = 0; k < V.rows(); k++){
    Vector3 a(V(k,0),V(k,1),V(k,2));
    for(uint j = 0; j < V.rows(); j++){
      Vector3 b(V(j,0),V(j,1),V(j,2));
      drawLineSegment(a,b);
    }
    //Vector3 b(V(k+1,0),V(k+1,1),V(k+1,2));
    //Vector3 c(V(k+2,0),V(k+2,1),V(k+2,2));
  }
      //drawLineSegment(cmplx.E.at(i).first, 
                      //cmplx.E.at(i).second);

  // int numpoints = 4;
  // coordT points[] = {0,0,0, 1,0,0, 0,1,0, 0,0,1};
  // int dim = 3;
  // char flags[25];
  // sprintf (flags, "qhull s FA");

  // qh_new_qhull(dim, numpoints, points, 0, flags, NULL, NULL);
  // qh_getarea(qh facet_list);
  // cout << qh totvol << endl;
  // cout << qh totarea << endl;
  // qh_freeqhull(!qh_ALL);
  //exit(0);
}

namespace cover{
  std::ostream& operator<< (std::ostream& out, const OpenSetConvex& set)
  {
    out << "C: " << set.region.ellipsoid.getC() << std::endl;
    out << "d: " << set.region.ellipsoid.getD() << std::endl;
    out << "A: " << set.region.polyhedron.getA() << std::endl;
    out << "b: " << set.region.polyhedron.getB() << std::endl;
    return out;
  }
}
