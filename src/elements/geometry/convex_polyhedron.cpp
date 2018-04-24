#include "convex_polyhedron.h"
#include "gui/common.h"
#include "klampt.h"
typedef Polyhedron_3::Vertex_handle                               Vertex_handle;
typedef Polyhedron_3::Facet                                       Facet;
typedef Polyhedron_3::Halfedge                                    Halfedge;
typedef Polyhedron_3::Vertex_const_iterator                       Vertex_const_iterator;
typedef Polyhedron_3::Vertex_iterator                             Vertex_iterator;
typedef Polyhedron_3::Facet_const_iterator                        Facet_const_iterator;
typedef Polyhedron_3::Facet_iterator                              Facet_iterator;
typedef Polyhedron_3::Halfedge_const_iterator                     Halfedge_const_iterator;
typedef Polyhedron_3::Halfedge_iterator                           Halfedge_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator            Halfedge_around_facet_circulator;
typedef Polyhedron_3::Halfedge_around_facet_const_circulator      Halfedge_around_facet_const_circulator;
typedef Polyhedron_3::Halfedge_handle                             Halfedge_handle;

using namespace GLDraw;

ConvexPolyhedron::ConvexPolyhedron(Eigen::MatrixXd A_, Eigen::VectorXd b_, Eigen::VectorXd center_):
  A(A_), b(b_), center(center_)
{
  bool IsCenterInsideRegion = ((A * center - b).maxCoeff()<=1e-10);

  if(!IsCenterInsideRegion){
    std::cout << "Center point of convex region is not inside convex region." << std::endl;
    std::cout << "Center point: " << center << std::endl;
    exit(0);
  }

  std::list<Plane_3> planes;
  for(uint k = 0; k < A.rows(); k++){
    Plane_3 plane(A(k,0), A(k,1), A(k,2), -b(k));
    planes.push_back(plane);
  }
  poly = new Polyhedron_3();

  Point_3 pcenter(center[0],center[1],center[2]);
  CGAL::halfspace_intersection_3(planes.begin(), planes.end(), *poly, pcenter);
}

ConvexPolyhedron::ConvexPolyhedron(Eigen::MatrixXd A_, Eigen::VectorXd b_):
  A(A_), b(b_)
{
  std::list<Plane_3> planes;
  for(uint k = 0; k < A.rows(); k++){
    Plane_3 plane(A(k,0), A(k,1), A(k,2), -b(k));
    planes.push_back(plane);
  }
  poly = new Polyhedron_3();
  CGAL::halfspace_intersection_3(planes.begin(), planes.end(), *poly);
  center = GetGeometricCenter();
}

ConvexPolyhedron::ConvexPolyhedron(Polyhedron_3 &poly_)
{
  poly = new Polyhedron_3(poly_);
}
ConvexPolyhedron::ConvexPolyhedron(const ConvexPolyhedron& cp_)
{
  poly = &cp_.GetCGALPolyhedronNonConst();
  A = cp_.GetA();
  b = cp_.GetB();
  center = cp_.GetCenter();
}
Eigen::MatrixXd ConvexPolyhedron::GetA() const
{
  return A;
}
Eigen::VectorXd ConvexPolyhedron::GetB() const
{
  return b;
}
Eigen::VectorXd ConvexPolyhedron::GetCenter() const
{
  return center;
}

Eigen::VectorXd ConvexPolyhedron::GetRandomPoint()
{
  if(!vrep_computed) vrep();

  Eigen::VectorXd r(3);

  Eigen::VectorXd center = GetGeometricCenter();
  double rk=0;
  for(uint k = 0; k < vertices.size(); k++){
    rk = rng_.uniformReal(rk,1);
    // std::cout << k << ":" << rk << std::endl;
    // std::cout << "center: " << center << std::endl;
    // std::cout << "vk: " << vertices.at(k) << std::endl;
    r += rk*(vertices.at(k) - center) + (1-rk)*center;
  }

  return r;
}

const Polyhedron_3& ConvexPolyhedron::GetCGALPolyhedron() const
{
  return *poly;
}
Polyhedron_3& ConvexPolyhedron::GetCGALPolyhedronNonConst() const
{
  return *poly;
}

std::vector<Eigen::VectorXd> ConvexPolyhedron::vrep()
{
  vertices.clear();
  for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
  {
    Point_3 p = v->point();
    Eigen::VectorXd q(3);
    for(uint k = 0; k < 3; k++) q[k] = CGAL::to_double(p[k]);
    vertices.push_back(q);
  }
  vrep_computed = true;
  return vertices;
}
std::pair<Eigen::MatrixXd, Eigen::VectorXd> ConvexPolyhedron::hrep() const
{
  return std::make_pair(A,b);
}
Eigen::VectorXd ConvexPolyhedron::GetGeometricCenter() const
{
  Eigen::VectorXd c(3); c.setZero();
  uint ctr = 0;
  for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
  {
    Point_3 p = v->point();
    double x = CGAL::to_double(p[0]);
    double y = CGAL::to_double(p[1]);
    double z = CGAL::to_double(p[2]);
    c[0] += x;
    c[1] += y;
    c[2] += z;
    ctr++;
  }
  if(ctr>0) c /= ctr;
  return c;
}

void ConvexPolyhedron::DrawGL(GUIState& state)
{
  if(state("draw_cover_vertices")){
    glPointSize(10);
    setColor(black);
    for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
    {
      Point_3 p = v->point();
      //Vector3 q(p[0],p[1],p[2]);
      double x = CGAL::to_double(p[0]);
      double y = CGAL::to_double(p[1]);
      double z = CGAL::to_double(p[2]);
      Vector3 q(x,y,z);
      drawPoint(q);
    }
  }

  if(state("draw_cover_edges")){
    glLineWidth(3);
    setColor(black);
    for ( Halfedge_const_iterator e = poly->halfedges_begin(); e != poly->halfedges_end(); ++e)
    {
      Point_3 v0 = e->vertex()->point();
      Point_3 v1 = e->next()->vertex()->point();
      //Vector3 q0(v0[0],v0[1],v0[2]);
      double x0 = CGAL::to_double(v0[0]);
      double y0 = CGAL::to_double(v0[1]);
      double z0 = CGAL::to_double(v0[2]);
      double x1 = CGAL::to_double(v1[0]);
      double y1 = CGAL::to_double(v1[1]);
      double z1 = CGAL::to_double(v1[2]);
      Vector3 q0(x0,y0,z0);
      Vector3 q1(x1,y1,z1);
      drawLineSegment(q0,q1);
    }
  }

  if(state("draw_cover_faces")){
    setColor(magenta);
    //uint ctr = 0;
    for ( Facet_const_iterator f = poly->facets_begin(); f != poly->facets_end(); ++f)
    {
      // bool active = IsActiveFacet(ctr++);
      // if(active) continue;
      Halfedge_around_facet_const_circulator fcirc = f->facet_begin();

      glBegin(GL_POLYGON);
      CGAL_For_all(fcirc, f->facet_begin())
      {
        glVertex3f(CGAL::to_double(fcirc->vertex()->point().x()),
                   CGAL::to_double(fcirc->vertex()->point().y()),
                   CGAL::to_double(fcirc->vertex()->point().z()));
      }
      glEnd();
    }
  }
  // if(state("draw_cover_active_faces")){
  //   setColor(black);
  //   uint ctr = 0;
  //   for ( Facet_const_iterator f = poly->facets_begin(); f != poly->facets_end(); ++f)
  //   {
  //     //bool active = IsActiveFacet(ctr++);
  //     //if(!active) continue;
  //     setColor(red);
  //     //std::cout << "face " << ctr << " active: " << (active?"yes":"no") << std::endl;
  //     Halfedge_around_facet_const_circulator fcirc = f->facet_begin();

  //     glBegin(GL_POLYGON);
  //     CGAL_For_all(fcirc, f->facet_begin())
  //     {
  //       // glVertex3f(fcirc->vertex()->point().x(),
  //       //            fcirc->vertex()->point().y(),
  //       //            fcirc->vertex()->point().z());
  //       glVertex3f(CGAL::to_double(fcirc->vertex()->point().x()),
  //                  CGAL::to_double(fcirc->vertex()->point().y()),
  //                  CGAL::to_double(fcirc->vertex()->point().z()));
  //     }
  //     glEnd();
  //   }
  // }

}
