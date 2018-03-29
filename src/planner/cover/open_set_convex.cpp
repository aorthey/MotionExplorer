#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"
#include "3rdparty/Polyhedron.h"
// extern "C" {
//   #include <qhull/qhull_a.h>
// }
// namespace SC{
// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Polyhedron_3.h>
// #include <CGAL/Surface_mesh.h>
// #include <CGAL/convex_hull_3.h>
// typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
// typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
// typedef K::Point_3                                Point_3;
// typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
// };

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
typedef Polyhedron_3::Vertex_handle                  Vertex_handle;
typedef Polyhedron_3::Facet                          Facet;
typedef Polyhedron_3::Halfedge                           Halfedge;
typedef Polyhedron_3::Vertex_iterator        Vertex_iterator;
typedef Polyhedron_3::Facet_iterator        Facet_iterator;
typedef Polyhedron_3::Halfedge_iterator        Halfedge_iterator;
//typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;

// #include <ompl/base/spaces/SE3StateSpace.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_):
  OpenSet(cspace_,s), region(region_)
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

  Eigen::MatrixXd A_eigen = region.getPolyhedron().getA();
  Eigen::VectorXd b_eigen = region.getPolyhedron().getB();

  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();

  Vector3 center(d_eigen[0],d_eigen[1],d_eigen[2]);
  Vector3 u(C_eigen(0,0),C_eigen(1,0),C_eigen(2,0));
  Vector3 v(C_eigen(0,1),C_eigen(1,1),C_eigen(2,1));
  Vector3 w(C_eigen(0,2),C_eigen(1,2),C_eigen(2,2));

  GLColor blue(0.1,0.1,0.9,1);
  GLColor black(0.2,0.2,0.2,1);
  GLColor magenta(0.8,0,0.8,0.3);
  setColor(blue);
  GLDraw::drawWireEllipsoid(center, u, v, w);

  std::cout << "distance seedpt to cvx set: " << (((A_eigen * d_eigen - b_eigen).maxCoeff()<=1e-10)?"contained":"NOT contained") << std::endl;

  //Both iris and Eigen::Polyhedron are wrong.
  //Must be an error in libcdd which messes things up. Maybe recompile with
  //exact and not floating based arithmetic?

  std::vector<Eigen::VectorXd> vxs = region.getPolyhedron().generatorPoints();
  for(uint k = 0; k < vxs.size(); k++){
    std::cout << vxs.at(k) << std::endl;
  }


  Eigen::Polyhedron P;
  P.vrep(A_eigen, b_eigen);
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> VV;
  VV = P.vrep();
  Eigen::MatrixXd V = VV.first;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << V << std::endl;
  exit(0);

  glPointSize(10);
  glLineWidth(3);
  setColor(black);

  std::vector<Point_3> points;
  for(uint k = 0; k < V.rows(); k++){
    Point_3 p(V(k,0),V(k,1),V(k,2));
    points.push_back(p);
  }
  Polyhedron_3 poly;
  
  CGAL::convex_hull_3(points.begin(), points.end(), poly);

  for ( Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
  {
    Point_3 p = v->point();
    Vector3 q(p[0],p[1],p[2]);
    drawPoint(q);
  }

  for ( Halfedge_iterator e = poly.halfedges_begin(); e != poly.halfedges_end(); ++e)
  {
    Point_3 v0 = e->vertex()->point();
    Point_3 v1 = e->next()->vertex()->point();
    Vector3 q0(v0[0],v0[1],v0[2]);
    Vector3 q1(v1[0],v1[1],v1[2]);
    drawLineSegment(q0,q1);
  }

  setColor(magenta);
  for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
  {
    if(f->is_triangle()){
      Point_3 v0 = f->halfedge()->vertex()->point();
      Point_3 v1 = f->halfedge()->next()->vertex()->point();
      Point_3 v2 = f->halfedge()->opposite()->vertex()->point();
      Vector3 q0(v0[0],v0[1],v0[2]);
      Vector3 q1(v1[0],v1[1],v1[2]);
      Vector3 q2(v2[0],v2[1],v2[2]);
      drawTriangle(q0,q1,q2);
    }else{
      std::cout << "cvx hull contains non-triangle facet" << std::endl;
    }
  }

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
