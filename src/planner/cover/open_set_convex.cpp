#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"
#include "3rdparty/Polyhedron.h"
extern "C" {
  #include <qhull/qhull_a.h>
}

// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSetConvex::OpenSetConvex(const ob::State *s, iris::IRISRegion region_):
  OpenSet(s), region(region_)
{
}

bool OpenSetConvex::IsInside(ob::State *sPrime)
{
  std::cout << "NYI" << std::endl;
  exit(0);
  return false;
}

void OpenSetConvex::DrawGL(GUIState&){
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

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
  GLColor black(0.2,0.2,0.2,1);
  GLColor magenta(0.8,0,0.8,0.3);
  setColor(blue);
  GLDraw::drawWireEllipsoid(center, u, v, w);
  setColor(grey);

  Eigen::Polyhedron P;
  P.vrep(A_eigen, b_eigen);
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> VV;
  VV = P.vrep();

  Eigen::MatrixXd V = VV.first;

  //for(uint k = 0; k < V.rows()-1; k++){
  //research of how we can visualize the cvx hull
  glPointSize(10);
  glLineWidth(2);
  drawPoint(center);
  setColor(black);

  // for(uint k = 0; k < V.rows(); k++){
  //   Vector3 a(V(0,0),V(0,1),V(0,2));
  //   Vector3 c(V(k,0),V(k,1),V(k,2));
  //   drawPoint(c);
  //   drawLineSegment(center,c);
  // }
  // Plane3D plane;
  // p->getPlane(0,plane);
  // glNormal3v(plane.normal);
  // glBegin(GL_TRIANGLE_FAN);
  // for(uint k = 0; k < V.rows(); k++){
  //   Vector3 a(V(0,0),V(0,1),V(0,2));
  //   glVertex3v(a);
  //   for(uint j = 1; j < V.rows()-1; j++){
  //     Vector3 b(V(j,0),V(j,1),V(j,2));
  //     Vector3 c(V(j+1,0),V(j+1,1),V(j+1,2));
  //     glVertex3v(b);
  //     glVertex3v(c);
  //   }
  // }
  // glEnd();


  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}

namespace cover{
  std::ostream& operator<< (std::ostream& out, const OpenSetConvex& set)
  {
    out << std::string(80, '-') << std::endl;
    out << "OpenSet Convex" << std::endl;
    out << std::string(80, '-') << std::endl;
    out << "Polyhedron:" << std::endl;
    out << "A: " << set.region.polyhedron.getA() << std::endl;
    out << "b: " << set.region.polyhedron.getB() << std::endl;
    out << "Ellipsoid:" << std::endl;
    out << "C: " << set.region.ellipsoid.getC() << std::endl;
    out << "d: " << set.region.ellipsoid.getD() << std::endl;
    return out;
  }
}
