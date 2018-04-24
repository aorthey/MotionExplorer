#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"
#include <fstream>

using namespace cover;

OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds):
  OpenSet(cspace_,s), region(region_)
{
  Eigen::MatrixXd A_poly = region.getPolyhedron().getA();
  Eigen::VectorXd b_poly = region.getPolyhedron().getB();

  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();

  polyhedron = new ConvexPolyhedron(A_poly, b_poly, d_eigen);
  polyhedron_bounds = new ConvexPolyhedron(bounds.getA(), bounds.getB(), d_eigen);

  ellipsoid = new Ellipsoid(C_eigen, d_eigen);

  nef_polyhedron = new NefPolyhedron( polyhedron_bounds );
  nef_polyhedron->SubtractObstacles(cspace);

  cvx_decomposition = nef_polyhedron->GetConvexDecomposition();
}

OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd center):
  OpenSet(cspace_,s)
{
  polyhedron = new ConvexPolyhedron(A, b, center);
}
OpenSetConvex::OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, ConvexPolyhedron *poly_):
  OpenSet(cspace_,s)
{
  polyhedron = new ConvexPolyhedron(*poly_); //deep copy
}

void OpenSetConvex::RandomState(ob::State *s)
{
  Eigen::VectorXd v = polyhedron->GetRandomPoint();
  Config q(3);
  for(uint j = 0; j < 3; j++) q(j) = v(j);
  cspace->ConfigToOMPLState(q, s);
}

bool OpenSetConvex::IsInside(ob::State *sPrime)
{
  std::cout << "NYI" << std::endl;
  exit(0);
  return false;
}

bool OpenSetConvex::IsSubsetOf(const cover::OpenSet *rhs_, double tolerance) const
{
  std::cout << "NYI" << std::endl;
  exit(0);
  // const cover::OpenSetConvex *rhs = dynamic_cast<const cover::OpenSetConvex*>(rhs_);
  // if(rhs==nullptr){
  //   std::cout << "could not cast rhs to convex open set." << std::endl;
  //   exit(0);
  // }

  // const Eigen::MatrixXd A_rhs = rhs->GetA();
  // const Eigen::VectorXd b_rhs = rhs->GetB();
  // std::vector<Eigen::VectorXd> vertices = vrep();

  // //polyhedron is contained in rhs IFF all vertices of polyhedron are contained
  // //in rhs

  // //(=>) if polyhedron is contained in rhs, then obviously all vertices needs to
  // //be contained.
  // //(<=) if all vertices of polyhedron are contained in rhs, then all the linear
  // //segments are contained by convexity. But then all faces are contained by
  // //convexity since they can be represented as linear segments between edge
  // //points. 
  // for(uint k = 0; k < vertices.size(); k++){
  //   if(!rhs->contains(vertices.at(k),tolerance)){
  //     return false;
  //   }
  // }
  // return true;

}
// bool OpenSetConvex::IsActiveFacet(uint k)
// {
//   const double EPSILON_EQ = 0.01;
//   Vector3 center = GetCenterOfFacet(k);
//   Eigen::VectorXd v(3);
//   for(uint k = 0; k < 3; k++) v[k] = center[k];

//   bool active = false;
//   for(uint j = 0; j < A_bounds.rows(); j++){
//     double d = A_bounds.row(j)*v - b_bounds(j);
//     if( fabs(d) <= EPSILON_EQ ){
//       active = true;
//     }
//   }
//   return active;
// }


// Vector3 OpenSetConvex::GetRandomPointOnFacet(uint k)
// {
//   std::vector<Vector3> fk = GetFacet(k);
//   Vector3 center = GetCenterOfFacet(k);

//   Vector3 v(center);
//   for(uint k = 0; k < fk.size(); k++){
//     v += rng_.uniform01()*(fk.at(k)-center);
//   }

//   return v;
// }
// Vector3 OpenSetConvex::GetCenterOfFacet(uint k)
// {
//   std::vector<Vector3> fk = GetFacet(k);
//   Vector3 center(0,0,0);
//   for(uint k = 0; k < fk.size(); k++){
//     center += fk.at(k);
//   }
//   center /= fk.size();
//   return center;
// }

// std::vector<Vector3> OpenSetConvex::GetFacet(uint k)
// {
//   uint N = poly.size_of_facets();
//   std::vector<Vector3> vertices;
//   if(k>=N) return vertices;
//   uint ctr = 0;

//   for ( Facet_iterator f = poly.facets_begin(); f != poly.facets_end(); ++f)
//   {
//     if(ctr==k){
//       Halfedge_around_facet_circulator fcirc = f->facet_begin();
//       CGAL_For_all(fcirc, f->facet_begin())
//       {
//         Vector3 v(CGAL::to_double(fcirc->vertex()->point().x()),
//                    CGAL::to_double(fcirc->vertex()->point().y()),
//                    CGAL::to_double(fcirc->vertex()->point().z()));

//         vertices.push_back(v);
//       }
//       return vertices;
//     }
//     ctr++;
//   }
//   std::cout << "WARNING: facet " << k << "/" << N << " not found." << std::endl;
//   return vertices;
// }

// uint OpenSetConvex::GetNumberOfInactiveFacets(){
//   uint ctr = 0;
//   for(uint k = 0; k < GetNumberOfFacets(); k++){
//     if(!IsActiveFacet(k)) ctr++;
//   }
//   return ctr;
// }
// uint OpenSetConvex::GetNumberOfFacets(){
//   return poly.size_of_facets();
// }

void OpenSetConvex::DrawGL(GUIState& state){
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  if(nef_polyhedron) nef_polyhedron->DrawGL(state);
  if(polyhedron) polyhedron->DrawGL(state);
  // for(uint k = 0; k < cvx_decomposition.size(); k++){
  //   cvx_decomposition.at(k).DrawGL(state);
  // }

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
}


std::ostream& OpenSetConvex::Print(std::ostream& out) const
{
  // out << std::string(80, '-') << std::endl;
  // out << "OpenSet Convex" << std::endl;
  // out << std::string(80, '-') << std::endl;
  // out << "Center: " << std::endl;
  // cspace->SpaceInformationPtr()->printState(sCenter, out);
  // out << "Polyhedron:" << std::endl;
  // out << "A: " << region.polyhedron.getA() << std::endl;
  // out << "b: " << region.polyhedron.getB() << std::endl;
  // out << "Ellipsoid:" << std::endl;
  // out << "C: " << region.ellipsoid.getC() << std::endl;
  // out << "d: " << region.ellipsoid.getD() << std::endl;
  return out;
}
