#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"

using namespace cover;

OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds):
  OpenSet(cspace_,s), region(region_)
{
  Eigen::MatrixXd A_eigen = region.getPolyhedron().getA();
  Eigen::VectorXd b_eigen = region.getPolyhedron().getB();
  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();
  bool IsCenterInsideRegion = ((A_eigen * d_eigen - b_eigen).maxCoeff()<=1e-10);

  if(!IsCenterInsideRegion){
    std::cout << "Center point of convex region is not inside convex region." << std::endl;
    std::cout << "Center point: " << d_eigen << std::endl;
    exit(0);
  }

  //compute CGAL polyhedron_3 from inflated iris region
  std::list<Plane_3> planes;
  for(uint k = 0; k < A_eigen.rows(); k++){
    Plane_3 plane(A_eigen(k,0), A_eigen(k,1), A_eigen(k,2), -b_eigen(k));
    planes.push_back(plane);
  }
  Point_3 pcenter(d_eigen[0],d_eigen[1],d_eigen[2]);

  CGAL::halfspace_intersection_3(planes.begin(), planes.end(), poly, pcenter);

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
    uint ctr = 0;
    for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
    {
      bool active = IsActiveFacet(ctr++);
      if(active) continue;
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
  if(state("draw_cover_active_faces")){
    setColor(black);
    uint ctr = 0;
    for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
    {
      bool active = IsActiveFacet(ctr++);
      if(!active) continue;
      //std::cout << "face " << ctr << " active: " << (active?"yes":"no") << std::endl;
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

bool OpenSetConvex::IsActiveFacet(uint k)
{
  const double EPSILON_EQ = 0.01;
  Vector3 center = GetCenterOfFacet(k);
  Eigen::VectorXd v(3);
  for(uint k = 0; k < 3; k++) v[k] = center[k];
  bool active = false;
  for(uint j = 0; j < A_bounds.rows(); j++){
    double d = A_bounds.row(j)*v - b_bounds(k);
    if( fabs(d) <= EPSILON_EQ ){
      active = true;
    }
  }

  if(!active){
    return false;
  }
  return true;
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
        glVertex3f(fcirc->vertex()->point().x(),
                   fcirc->vertex()->point().y(),
                   fcirc->vertex()->point().z());

        Vector3 v(fcirc->vertex()->point().x(),
                   fcirc->vertex()->point().y(),
                   fcirc->vertex()->point().z());
        vertices.push_back(v);
      }
      return vertices;
    }
    ctr++;
  }
  std::cout << "WARNING: facet " << k << "/" << N << " not found." << std::endl;
  return vertices;
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
