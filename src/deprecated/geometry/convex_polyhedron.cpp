#include "convex_polyhedron.h"
#include "gui/common.h"
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

#include <CGAL/point_generators_3.h>

using namespace GLDraw;
#define DEBUG_CVX 0

ConvexPolyhedron::ConvexPolyhedron(Eigen::MatrixXd A_, Eigen::VectorXd b_, Eigen::VectorXd center_):
  A(A_), b(b_), center(center_)
{
  if(!IsInside(center))
  {
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

  hrep_computed = true;
  vrep_computed = false;
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
  hrep_computed = true;
  vrep_computed = false;
}

ConvexPolyhedron::ConvexPolyhedron(Polyhedron_3 &poly_)
{
  poly = new Polyhedron_3(poly_);
  hrep_computed = false;
  vrep_computed = false;
}
ConvexPolyhedron::ConvexPolyhedron(const ConvexPolyhedron& cp_)
{
  poly = &cp_.GetCGALPolyhedronNonConst();
  A = cp_.GetA();
  b = cp_.GetB();
  center = cp_.GetCenter();
  hrep_computed = cp_.hrep_computed;
  vrep_computed = cp_.vrep_computed;
}
Point_3 ConvexPolyhedron::EigenToCGAL(const Eigen::VectorXd &v) const
{
  assert(v.size()==3);
  Point_3 p(v[0],v[1],v[2]);
  return p;
}
Vector3 ConvexPolyhedron::CGALToVector3(const Point_3 &p) const
{
  double x = CGAL::to_double(p[0]);
  double y = CGAL::to_double(p[1]);
  double z = CGAL::to_double(p[2]);
  Vector3 q(x,y,z);
  return q;
}
Eigen::VectorXd ConvexPolyhedron::CGALToEigen(const Point_3 &p) const
{
  Eigen::VectorXd r(3);
  double x = CGAL::to_double(p[0]);
  double y = CGAL::to_double(p[1]);
  double z = CGAL::to_double(p[2]);
  r[0] = x;
  r[1] = y;
  r[2] = z;
  return r;
}
std::vector<Eigen::VectorXd> ConvexPolyhedron::vrep()
{
  vertices.clear();
  for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
  {
    Point_3 p = v->point();
    Eigen::VectorXd q = CGALToEigen(p);
    //for(uint k = 0; k < 3; k++) q[k] = CGAL::to_double(p[k]);
    vertices.push_back(q);
  }
  vrep_computed = true;
  return vertices;
}


struct Plane_equation {
    template <class Facet>
    typename Facet::Plane_3 operator()( Facet& f) {
        typename Facet::Halfedge_handle h = f.halfedge();
        typedef typename Facet::Plane_3  Plane;
        return Plane( h->vertex()->point(),
                      h->next()->vertex()->point(),
                      h->next()->next()->vertex()->point());
    }
};

std::pair<Eigen::MatrixXd, Eigen::VectorXd> ConvexPolyhedron::hrep()
{
  if(!hrep_computed){
    A.resize(poly->size_of_facets(),3);
    b.resize(poly->size_of_facets());
    uint ctr = 0;
    //std::cout << "poly has " << poly->size_of_facets() << " facets" << std::endl;


    std::transform( poly->facets_begin(), poly->facets_end(), poly->planes_begin(),
                            Plane_equation());
    // CGAL::set_pretty_mode( std::cout);
    // std::copy( poly->planes_begin(), poly->planes_end(),
    //                    std::ostream_iterator<Plane_3>( std::cout, "\n"));
    // exit(0);

    // for ( Facet_const_iterator f = poly->facets_begin(); f != poly->facets_end(); ++f)
    // {
    //   Halfedge_around_facet_const_circulator fcirc = f->facet_begin();
    //   CGAL_For_all(fcirc, f->facet_begin())
    //   {
    //     double pa = CGAL::to_double(fcirc->vertex()->point().x());
    //     double pb = CGAL::to_double(fcirc->vertex()->point().y());
    //     double pc = CGAL::to_double(fcirc->vertex()->point().z());
    //     std::cout << "[vertex] " << pa << "," << pb << "," << pc << std::endl;
    //   }
    // }
    for ( Facet_const_iterator f = poly->facets_begin(); f != poly->facets_end(); ++f)
    {
      Plane_3 plane = f->plane();
      Eigen::VectorXd ap(3);
      double pa = CGAL::to_double(plane.a());
      double pb = CGAL::to_double(plane.b());
      double pc = CGAL::to_double(plane.c());
      double pd = CGAL::to_double(plane.d());
      //double d = sqrtf(pa*pa+pb*pb+pc*pc);
      A(ctr,0) = pa;
      A(ctr,1) = pb;
      A(ctr,2) = pc;
      //std::cout << "[plane] " << pa << "," << pb << "," << pc << "," << pd << std::endl;
      b(ctr) = -pd;
      ctr++;

    }
    center = GetGeometricCenter();
    hrep_computed = true;
  }
  return std::make_pair(A,b);
}
Eigen::VectorXd ConvexPolyhedron::GetRandomPoint()
{
  // if(!vrep_computed) vrep();

  // Eigen::VectorXd center = GetGeometricCenter();

  // CGAL::Random_points_in_triangle_mesh_3<Polyhedron_3> g(*poly);
  // //std::vector<Point> points;
  // //CGAL::cpp11::copy_n(g, 1, std::back_inserter(points));
  // Point_3 p = *g++;
  // Eigen::VectorXd r = CGALToEigen(p);
  // double rk = rng_.uniformReal(0,1);

  // //get sample on line $r \in (center,r)$
  // Eigen::VectorXd interior_r = rk*(r - center) + center;
  // return interior_r;
  return GetRandomPoint_RejectionSampling();
}

void ConvexPolyhedron::test_hrep()
{
  if(!hrep_computed) hrep();
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  std::cout << "test halfspace representation" << std::endl;
  //center (needs to be inside)
  Eigen::VectorXd center = GetGeometricCenter();
  std::cout << "[center] " << center.format(CommaInitFmt) << (IsInside(center)?"OK (inside)":">>Error<<") << std::endl;
  //vertices (need to be inside)
  for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
  {
    Eigen::VectorXd qv = CGALToEigen(v->point());
    std::cout << "[vertex] " << qv.format(CommaInitFmt) << (IsInside(qv)?"OK (inside)":">>Error<<") << std::endl;
  }
  //extended vertices (should be outside the polyhedron)
  double epsilon = 1e-5;
  for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
  {

    Eigen::VectorXd qd = CGALToEigen(v->point())-center;
    Eigen::VectorXd qdn = qd.normalized();
    Eigen::VectorXd qv = center + qd + epsilon*qdn;
    std::cout << "[extended-vertex] " << qv.format(CommaInitFmt) << (!IsInside(qv)?"OK (outside)":">>Error<<") << std::endl;
  }
}

Eigen::VectorXd ConvexPolyhedron::GetRandomPoint_RejectionSampling()
{
  if(!vrep_computed) vrep();
  if(!bbox_computed) bbox();

  uint iter = 0;
  while(iter++<10){
    Nsamples++;
    Eigen::VectorXd r(3);
    for(uint k = 0; k < 3; k++){
      r[k] = rng_.uniformReal(bbox_min[k],bbox_max[k]);
    }
    if(IsInside(r))
    {
      return r;
    }
    Nsamples_rejected++;
  }
  std::cout << "[WARNING] could not sample polyhedron" << std::endl;
  std::cout << *this << std::endl;
  double r_rate = (double)Nsamples_rejected/(double)Nsamples;
  std::cout << "Rejection rate: " << r_rate << " (" << Nsamples_rejected << "/" << Nsamples << ")" << std::endl;
  if(IsInside(center)){
    return center;
  }
  return vertices.at(0);

}


bool ConvexPolyhedron::IsInside(const Eigen::VectorXd &p)
{
  if(!hrep_computed) hrep();
  return ((A * p - b).maxCoeff()<=1e-10);
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

const Polyhedron_3& ConvexPolyhedron::GetCGALPolyhedron() const
{
  return *poly;
}
Polyhedron_3& ConvexPolyhedron::GetCGALPolyhedronNonConst() const
{
  return *poly;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> ConvexPolyhedron::bbox()
{
  if(!vrep_computed) vrep();

  //std::cout << *this << std::endl;

  bbox_min.resize(3); bbox_min.setZero();
  bbox_max.resize(3); bbox_max.setZero();

  if(vertices.size()>0){
    bbox_max = vertices.at(0);
    bbox_min = vertices.at(0);
  }

  for(uint k = 0; k < vertices.size(); k++){
    for(uint j = 0; j < 3; j++){
      bbox_min[j] = min(bbox_min[j], vertices.at(k)[j]);
      bbox_max[j] = max(bbox_max[j], vertices.at(k)[j]);
    }
  }
  bbox_computed = true;
  return std::make_pair(bbox_min, bbox_max);
}

Eigen::VectorXd ConvexPolyhedron::GetGeometricCenter() const
{
  Eigen::VectorXd c(3); c.setZero();
  uint ctr = 0;
  for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
  {
    Point_3 p = v->point();
    Eigen::VectorXd cp = CGALToEigen(p);
    c += cp;
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
      Vector3 q = CGALToVector3(v->point());
      drawPoint(q);
    }
  }
  if(state("draw_cover_geometric_center")){
    glPointSize(10);
    setColor(black);
    Point_3 center = EigenToCGAL(GetGeometricCenter());
    Vector3 q0 = CGALToVector3(center);
    for ( Vertex_const_iterator v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
    {
      Point_3 p = v->point();
      Vector3 q1 = CGALToVector3(p);
      drawPoint(q1);
      drawLineSegment(q0,q1);
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
      // double x0 = CGAL::to_double(v0[0]);
      // double y0 = CGAL::to_double(v0[1]);
      // double z0 = CGAL::to_double(v0[2]);
      // double x1 = CGAL::to_double(v1[0]);
      // double y1 = CGAL::to_double(v1[1]);
      // double z1 = CGAL::to_double(v1[2]);
      // Vector3 q0(x0,y0,z0);
      // Vector3 q1(x1,y1,z1);

      Vector3 q0 = CGALToVector3(v0);
      Vector3 q1 = CGALToVector3(v1);
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
std::ostream& operator<< (std::ostream& out, const ConvexPolyhedron& cvxp)
{
  out << "[ConvexPolyhedron" << std::endl;
  out << " -- vertices  : " << cvxp.poly->size_of_vertices() << std::endl;
  out << " -- halfedges : " << cvxp.poly->size_of_halfedges() << std::endl;
  out << " -- faces     : " << cvxp.poly->size_of_facets() << std::endl;

  out << " -- (hrep) " << (cvxp.hrep_computed?"OK":" not computed") << std::endl;
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "       [", "]");
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

  if(cvxp.hrep_computed){
    out << cvxp.A.format(CleanFmt) << std::endl;
    out << cvxp.b.format(CleanFmt) << std::endl;
    Eigen::VectorXd c(3);c.setZero();
    bool CisInside = ((cvxp.A * c - cvxp.b).maxCoeff()<=1e-10);
    out << "       zero: " << (CisInside?"inside":"outside") << std::endl;
    Eigen::VectorXd p = cvxp.GetGeometricCenter();
    bool PisInside = ((cvxp.A * p - cvxp.b).maxCoeff()<=1e-10);
    out << "       center: " << p.format(CommaInitFmt) << std::endl;
    out << "       center: " << (PisInside?"inside":"outside") << std::endl;
  }

  out << " -- (vrep) " << (cvxp.vrep_computed?"OK":" not computed") << std::endl;
  if(cvxp.vrep_computed){
    out << "       vertices  : " << cvxp.vertices.size() << std::endl;
    for ( Vertex_const_iterator v = cvxp.poly->vertices_begin(); v != cvxp.poly->vertices_end(); ++v)
    {
      Vector3 q = cvxp.CGALToVector3(v->point());
      out << "       [vertex] " << q << std::endl;
    }
  }

  out << " -- (bbox) " << (cvxp.bbox_computed?"OK":" not computed") << std::endl;
  if(cvxp.bbox_computed){
    out << "       bbox_min " << cvxp.bbox_min.format(CommaInitFmt) << std::endl;
    out << "       bbox_max " << cvxp.bbox_max.format(CommaInitFmt)  << std::endl;
  }
  return out;
}
