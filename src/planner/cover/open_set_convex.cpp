#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef K::Point_3                                            Point_3;
typedef K::Plane_3                                            Plane_3;
typedef CGAL::Polyhedron_3<K>                                 Polyhedron_3;
typedef Polyhedron_3::Vertex_handle                           Vertex_handle;
typedef Polyhedron_3::Facet                                   Facet;
typedef Polyhedron_3::Halfedge                                Halfedge;
typedef Polyhedron_3::Vertex_iterator                         Vertex_iterator;
typedef Polyhedron_3::Facet_iterator                          Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator        Halfedge_around_facet_circulator;
typedef Polyhedron_3::Halfedge_handle                         Halfedge_handle;
typedef Polyhedron_3::Halfedge_iterator                       Halfedge_iterator;

using namespace cover;

OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_):
  OpenSet(cspace_,s), region(region_)
{
  // Eigen::MatrixXd A_eigen = region.getPolyhedron().getA();
  // Eigen::VectorXd b_eigen = region.getPolyhedron().getB();
  // Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  // Eigen::VectorXd d_eigen = region.getEllipsoid().getD();
  //std::cout << "seed point is member of Polyhedron? " << (((A_eigen * d_eigen - b_eigen).maxCoeff()<=1e-10)?"YES":"NO") << std::endl;
}

bool OpenSetConvex::IsInside(ob::State *sPrime)
{
  std::cout << "NYI" << std::endl;
  exit(0);
  return false;
}

void OpenSetConvex::DrawGL(GUIState& state){
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  Eigen::MatrixXd A_eigen = region.getPolyhedron().getA();
  Eigen::VectorXd b_eigen = region.getPolyhedron().getB();
  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();

  GLColor grey(0.7,0.7,0.7,1);
  GLColor black(0.2,0.2,0.2,1);
  GLColor magenta(0.8,0,0.8,0.3);

  if(state("draw_cover_ellipsoid")){
    glLineWidth(1);
    setColor(grey);

    Vector3 center(d_eigen[0],d_eigen[1],d_eigen[2]);
    Vector3 u(C_eigen(0,0),C_eigen(1,0),C_eigen(2,0));
    Vector3 v(C_eigen(0,1),C_eigen(1,1),C_eigen(2,1));
    Vector3 w(C_eigen(0,2),C_eigen(1,2),C_eigen(2,2));
    GLDraw::drawWireEllipsoid(center, u, v, w);
  }

  std::list<Plane_3> planes;
  for(uint k = 0; k < A_eigen.rows(); k++){
    Plane_3 plane(A_eigen(k,0), A_eigen(k,1), A_eigen(k,2), -b_eigen(k));
    planes.push_back(plane);
  }
  Point_3 pcenter(d_eigen[0],d_eigen[1],d_eigen[2]);

  Polyhedron_3 poly;
  CGAL::halfspace_intersection_3(planes.begin(), planes.end(), poly, pcenter);

  if(state("draw_cover_vertices")){
    glPointSize(10);
    setColor(black);
    for ( Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
    {
      Point_3 p = v->point();
      Vector3 q(p[0],p[1],p[2]);
      drawPoint(q);
    }
  }

  if(state("draw_cover_edges")){
    glLineWidth(3);
    setColor(black);
    for ( Halfedge_iterator e = poly.halfedges_begin(); e != poly.halfedges_end(); ++e)
    {
      Point_3 v0 = e->vertex()->point();
      Point_3 v1 = e->next()->vertex()->point();
      Vector3 q0(v0[0],v0[1],v0[2]);
      Vector3 q1(v1[0],v1[1],v1[2]);
      drawLineSegment(q0,q1);
    }
  }

  if(state("draw_cover_faces")){
    setColor(magenta);
    for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
    {
      Halfedge_around_facet_circulator fcirc = f->facet_begin();

      glBegin(GL_POLYGON);
      CGAL_For_all(fcirc, f->facet_begin())
      {
        glVertex3f(fcirc->vertex()->point().x(),
                   fcirc->vertex()->point().y(),
                   fcirc->vertex()->point().z());
      }
      glEnd();
    }
  }

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}

std::ostream& OpenSetConvex::Print(std::ostream& out) const
{
  out << std::string(80, '-') << std::endl;
  out << "OpenSet Convex" << std::endl;
  out << std::string(80, '-') << std::endl;
  out << "Center: " << std::endl;
  cspace->SpaceInformationPtr()->printState(sCenter, out);
  out << "Polyhedron:" << std::endl;
  out << "A: " << region.polyhedron.getA() << std::endl;
  out << "b: " << region.polyhedron.getB() << std::endl;
  out << "Ellipsoid:" << std::endl;
  out << "C: " << region.ellipsoid.getC() << std::endl;
  out << "d: " << region.ellipsoid.getD() << std::endl;
  return out;
}
