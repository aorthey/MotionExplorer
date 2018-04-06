#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"

using namespace cover;

OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds):
  OpenSet(cspace_,s), region(region_)
{
  A_poly = region.getPolyhedron().getA();
  b_poly = region.getPolyhedron().getB();
  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();
  bool IsCenterInsideRegion = ((A_poly * d_eigen - b_poly).maxCoeff()<=1e-10);

  if(!IsCenterInsideRegion){
    std::cout << "Center point of convex region is not inside convex region." << std::endl;
    std::cout << "Center point: " << d_eigen << std::endl;
    exit(0);
  }

  //compute CGAL polyhedron_3 from inflated iris region
  std::list<Plane_3> planes;
  for(uint k = 0; k < A_poly.rows(); k++){
    Plane_3 plane(A_poly(k,0), A_poly(k,1), A_poly(k,2), -b_poly(k));
    planes.push_back(plane);
  }
  Point_3 pcenter(d_eigen[0],d_eigen[1],d_eigen[2]);

  CGAL::halfspace_intersection_3(planes.begin(), planes.end(), poly, pcenter);

  Nef_polyhedron nef_poly_tmp(poly);
  nef_poly = nef_poly_tmp;

  //compute CGAL polyhedron_3 from bounds
  A_bounds = bounds.getA();
  b_bounds = bounds.getB();
  std::list<Plane_3> bound_planes;
  for(uint k = 0; k < A_bounds.rows(); k++){
    Plane_3 plane(A_bounds(k,0), A_bounds(k,1), A_bounds(k,2), -b_bounds(k));
    bound_planes.push_back(plane);
  }

  //pcenter is by definition an interior point of bounds
  CGAL::halfspace_intersection_3(bound_planes.begin(), bound_planes.end(), poly_bounds, pcenter);
}

bool OpenSetConvex::IsInside(ob::State *sPrime)
{
  std::cout << "NYI" << std::endl;
  exit(0);
  return false;
}

void OpenSetConvex::RemoveIntersection(const cover::OpenSet *rhs_)
{
  const cover::OpenSetConvex *rhs = dynamic_cast<const cover::OpenSetConvex*>(rhs_);
  if(rhs==nullptr){
    std::cout << "could not cast rhs to convex open set." << std::endl;
    exit(0);
  }
  nef_poly -= rhs->GetNefPolyhedron();
}

void OpenSetConvex::DrawGLNefPolyhedron(GUIState& state){
  if(state("draw_cover_edges")){
    glLineWidth(3);
    setColor(black);
    for ( Nef_halfedge_const_iterator e = nef_poly.halfedges_begin(); e != nef_poly.halfedges_end(); ++e)
    {
      Point_3 v0 = e->source()->point();
      Point_3 v1 = e->target()->point();
      //Vector3 q0(v0[0],v0[1],v0[2]);
      double x0 = CGAL::to_double(v0[0]);
      double y0 = CGAL::to_double(v0[1]);
      double z0 = CGAL::to_double(v0[2]);
      double x1 = CGAL::to_double(v1[0]);
      double y1 = CGAL::to_double(v1[1]);
      double z1 = CGAL::to_double(v1[2]);
      Vector3 q0(x0,y0,z0);
      Vector3 q1(x1,y1,z1);
      drawPoint(q0);
      drawLineSegment(q0,q1);
    }
  }
}

void OpenSetConvex::DrawGLPolyhedron(GUIState& state){
  if(state("draw_cover_vertices")){
    glPointSize(10);
    setColor(black);
    for ( Vertex_const_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
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
    for ( Halfedge_iterator e = poly.halfedges_begin(); e != poly.halfedges_end(); ++e)
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
    uint ctr = 0;
    for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
    {
      bool active = IsActiveFacet(ctr++);
      if(active) continue;
      Halfedge_around_facet_circulator fcirc = f->facet_begin();

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
  if(state("draw_cover_active_faces")){
    setColor(black);
    uint ctr = 0;
    for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
    {
      // Vector3 v = GetCenterOfFacet(ctr);
      // setColor(red);
      // glPointSize(20);
      // drawPoint(v);
      bool active = IsActiveFacet(ctr++);
      if(!active) continue;
      setColor(red);
      //std::cout << "face " << ctr << " active: " << (active?"yes":"no") << std::endl;
      Halfedge_around_facet_circulator fcirc = f->facet_begin();

      glBegin(GL_POLYGON);
      CGAL_For_all(fcirc, f->facet_begin())
      {
        // glVertex3f(fcirc->vertex()->point().x(),
        //            fcirc->vertex()->point().y(),
        //            fcirc->vertex()->point().z());
        glVertex3f(CGAL::to_double(fcirc->vertex()->point().x()),
                   CGAL::to_double(fcirc->vertex()->point().y()),
                   CGAL::to_double(fcirc->vertex()->point().z()));
      }
      glEnd();
    }
  }

}

void OpenSetConvex::DrawGLEllipsoid(GUIState& state)
{
  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();

  if(state("draw_cover_ellipsoid")){
    glLineWidth(1);
    setColor(grey);

    Vector3 center(d_eigen[0],d_eigen[1],d_eigen[2]);
    Vector3 u(C_eigen(0,0),C_eigen(1,0),C_eigen(2,0));
    Vector3 v(C_eigen(0,1),C_eigen(1,1),C_eigen(2,1));
    Vector3 w(C_eigen(0,2),C_eigen(1,2),C_eigen(2,2));
    GLDraw::drawWireEllipsoid(center, u, v, w);
  }
}

void OpenSetConvex::DrawGL(GUIState& state){
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  DrawGLEllipsoid(state);
  DrawGLNefPolyhedron(state);

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}

bool OpenSetConvex::IsActiveFacet(uint k)
{
  const double EPSILON_EQ = 0.01;
  Vector3 center = GetCenterOfFacet(k);
  Eigen::VectorXd v(3);
  for(uint k = 0; k < 3; k++) v[k] = center[k];

  bool active = false;
  for(uint j = 0; j < A_bounds.rows(); j++){
    double d = A_bounds.row(j)*v - b_bounds(j);
    if( fabs(d) <= EPSILON_EQ ){
      active = true;
    }
  }
  return active;
}

std::vector<Eigen::VectorXd> OpenSetConvex::vrep() const
{
  std::vector<Eigen::VectorXd> vertices;
  for ( Vertex_const_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
  {
    Point_3 p = v->point();
    Eigen::VectorXd q(3);
    for(uint k = 0; k < 3; k++) q[k] = CGAL::to_double(p[k]);
    vertices.push_back(q);
  }
  return vertices;
}
const Nef_polyhedron& OpenSetConvex::GetNefPolyhedron() const
{
  return nef_poly;
}
const Polyhedron_3& OpenSetConvex::GetPolyhedron() const
{
  return poly;
}

const Eigen::MatrixXd OpenSetConvex::GetA() const
{
  return A_poly;
}

const Eigen::VectorXd OpenSetConvex::GetB() const
{
  return b_poly;
}

bool OpenSetConvex::contains(const Eigen::VectorXd& point, double tolerance) const
{
  return (A_poly * point - b_poly).maxCoeff() <= tolerance;
}

bool OpenSetConvex::IsSubsetOf(const cover::OpenSet *rhs_, double tolerance) const
{
  const cover::OpenSetConvex *rhs = dynamic_cast<const cover::OpenSetConvex*>(rhs_);
  if(rhs==nullptr){
    std::cout << "could not cast rhs to convex open set." << std::endl;
    exit(0);
  }

  const Eigen::MatrixXd A_rhs = rhs->GetA();
  const Eigen::VectorXd b_rhs = rhs->GetB();
  std::vector<Eigen::VectorXd> vertices = vrep();

  //polyhedron is contained in rhs IFF all vertices of polyhedron are contained
  //in rhs

  //(=>) if polyhedron is contained in rhs, then obviously all vertices needs to
  //be contained.
  //(<=) if all vertices of polyhedron are contained in rhs, then all the linear
  //segments are contained by convexity. But then all faces are contained by
  //convexity since they can be represented as linear segments between edge
  //points. 
  for(uint k = 0; k < vertices.size(); k++){
    if(!rhs->contains(vertices.at(k),tolerance)){
      return false;
    }
  }
  return true;

}

Vector3 OpenSetConvex::GetRandomPointOnFacet(uint k)
{
  std::vector<Vector3> fk = GetFacet(k);
  Vector3 center = GetCenterOfFacet(k);

  Vector3 v(center);
  for(uint k = 0; k < fk.size(); k++){
    v += rng_.uniform01()*(fk.at(k)-center);
  }

  return v;
}
Vector3 OpenSetConvex::GetCenterOfFacet(uint k)
{
  std::vector<Vector3> fk = GetFacet(k);
  Vector3 center(0,0,0);
  for(uint k = 0; k < fk.size(); k++){
    center += fk.at(k);
  }
  center /= fk.size();
  return center;
}

std::vector<Vector3> OpenSetConvex::GetFacet(uint k)
{
  uint N = poly.size_of_facets();
  std::vector<Vector3> vertices;
  if(k>=N) return vertices;
  uint ctr = 0;

  for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
  {
    if(ctr==k){
      Halfedge_around_facet_circulator fcirc = f->facet_begin();
      CGAL_For_all(fcirc, f->facet_begin())
      {
        Vector3 v(CGAL::to_double(fcirc->vertex()->point().x()),
                   CGAL::to_double(fcirc->vertex()->point().y()),
                   CGAL::to_double(fcirc->vertex()->point().z()));

        // Vector3 v(fcirc->vertex()->point().x(),
        //            fcirc->vertex()->point().y(),
        //            fcirc->vertex()->point().z());
        vertices.push_back(v);
      }
      return vertices;
    }
    ctr++;
  }
  std::cout << "WARNING: facet " << k << "/" << N << " not found." << std::endl;
  return vertices;
}

uint OpenSetConvex::GetNumberOfInactiveFacets(){
  uint ctr = 0;
  for(uint k = 0; k < GetNumberOfFacets(); k++){
    if(!IsActiveFacet(k)) ctr++;
  }
  return ctr;
}
uint OpenSetConvex::GetNumberOfFacets(){
  return poly.size_of_facets();
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
