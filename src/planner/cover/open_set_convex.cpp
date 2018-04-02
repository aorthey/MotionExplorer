#include "common.h"
#include "open_set_convex.h"
#include "gui/drawMotionPlanner.h"
#include "gui/colors.h"
#include "3rdparty/vrep.h"

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

#include <stdio.h>
#undef PPL_HAVE_TYPEOF
#include "ppl.hh"
#include "iris/iris.h"

// Note: this test is no longer in use (and never really worked anyway). I'm just keeping it around in case I ever want to try again with PPL. 

using namespace Parma_Polyhedra_Library;
using namespace Eigen;

struct Floating_Real_Open_Interval_Info_Policy {
  const_bool_nodef(store_special, false);
  const_bool_nodef(store_open, true);
  const_bool_nodef(cache_empty, true);
  const_bool_nodef(cache_singleton, true);
  const_bool_nodef(cache_normalized, false);
  const_int_nodef(next_bit, 0);
  const_bool_nodef(may_be_empty, true);
  const_bool_nodef(may_contain_infinity, false);
  const_bool_nodef(check_empty_result, false);
  const_bool_nodef(check_inexact, false);
};

typedef Interval_Info_Bitset<unsigned int,
                             Floating_Real_Open_Interval_Info_Policy> Floating_Real_Open_Interval_Info;

//! The type of an interval with floating point boundaries.
typedef Interval<double,
                 Floating_Real_Open_Interval_Info> FP_Interval;

//! The type of an interval linear form.
typedef Linear_Form<FP_Interval> FP_Linear_Form;

//! The type of an interval abstract store.
typedef Box<FP_Interval> FP_Interval_Abstract_Store;

void getGenerators(const Polyhedron* self) {
  const int dim = self->getDimension();
  NNC_Polyhedron ppl_polyhedron(dim);
  std::vector<Variable> vars;
  for (int i=0; i < dim; i++) {
    Variable v(i);
    vars.push_back(v);
  }
  for (int i=0; i < self->getNumberOfConstraints(); i++) {
    FP_Linear_Form expr;
    for (int j=0; j < dim; j++) {
      expr += FP_Linear_Form(self->getA()(i,j) * vars[j]);
    }
    expr.print();
    printf("\n");
    FP_Linear_Form right(self->getB()(i) + 0.0 * vars[0]);
    right.print();
    printf("\n");
    ppl_polyhedron.refine_with_linear_form_inequality(expr, right);
    // ppl_polyhedron.add_constraint(expr <= Linear_Form<double>(self->getB()(i)));
  }

  auto generators = ppl_polyhedron.generators();
  generators.print();
  std::cout << std::endl;
  for (auto gen = generators.begin(); gen != generators.end(); ++gen) {
    gen->print();
    // for (int i=0; i < dim; i++) {
    //   printf("%f", static_cast<const int>(gen->coefficient(vars[i])));
    // }
    printf("\n");
    // printf(" %d\n", static_cast<const int> gen->divisor());
  }

  ppl_polyhedron.constraints().print();
}














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
  Eigen::MatrixXd A_eigen = region.getPolyhedron().getA();
  Eigen::VectorXd b_eigen = region.getPolyhedron().getB();
  Eigen::MatrixXd C_eigen = region.getEllipsoid().getC();
  Eigen::VectorXd d_eigen = region.getEllipsoid().getD();
  std::cout << "seed point is member of Polyhedron? " << (((A_eigen * d_eigen - b_eigen).maxCoeff()<=1e-10)?"YES":"NO") << std::endl;
  for(uint k = 0; k < A_eigen.rows(); k++){
    for(uint j = 0; j < A_eigen.cols(); j++){
      if(abs(A_eigen(k,j)) <= 1e-6){
        //dirty hack: points below 1e-6 are treated as zero by cdd and causes
        //some hyperplanes to be ignored (not sure why). this trick here seems
        //to solve the problem.
        //A_eigen(k,j) += boost::math::sign(A_eigen(k,j))*1e-6;
        A_eigen(k,j) -= 2*1e-6;
      }
      A_eigen(k,j) += 2*1e-6;
    }
  }
  region.polyhedron.setA(A_eigen);
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

  GLColor grey(0.7,0.7,0.7,1);
  GLColor black(0.2,0.2,0.2,1);
  GLColor magenta(0.8,0,0.8,0.3);
  setColor(grey);
  GLDraw::drawWireEllipsoid(center, u, v, w);

  getGenerators(&region.getPolyhedron());

  //Both iris and Eigen::Polyhedron are wrong.
  //Must be an error in libcdd which messes things up. Maybe recompile with
  //exact and not floating based arithmetic?
  std::vector<Eigen::VectorXd> vxs = region.getPolyhedron().generatorPoints();
  for(uint k = 0; k < vxs.size(); k++){
    std::cout << vxs.at(k) << std::endl;
  }
  exit(0);

  // VertexRepresentation vrep(A_eigen, b_eigen);
  // Eigen::MatrixXd V = vrep.GetVertices();

  // Eigen::Polyhedron P;
  // P.vrep(A_eigen, b_eigen);
  // std::pair<Eigen::MatrixXd, Eigen::VectorXd> VV;
  // VV = P.vrep();
  // Eigen::MatrixXd V = VV.first;
  // std::cout << std::string(80, '-') << std::endl;
  // std::cout << V << std::endl;
  //exit(0);

  glPointSize(10);
  glLineWidth(3);
  setColor(black);

  std::vector<Point_3> points;
  for(uint k = 0; k < vxs.size(); k++){
    Eigen::VectorXd V = vxs.at(k);
    Point_3 p(V(0),V(1),V(2));
    points.push_back(p);
  }
  // for(uint k = 0; k < V.rows(); k++){
  //   Point_3 p(V(k,0),V(k,1),V(k,2));
  //   points.push_back(p);
  // }
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
