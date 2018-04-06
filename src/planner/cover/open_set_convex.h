#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <iris/iris.h>
#include <ompl/util/RandomNumbers.h>
namespace ob = ompl::base;

class CSpaceOMPL;

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Gmpq.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel   Kernel;
//typedef CGAL::Exact_integer  NT;
//typedef CGAL::Extended_homogeneous<NT>  Kernel;
//struct Kernel : public CGAL::Extended_homogeneous<NT> {};
//typedef CGAL::Extended_homogeneous<leda_real>  Kernel;
// typedef CGAL::Gmpq  FT;
// typedef CGAL::Cartesian_d<FT>  Kernel;

typedef Kernel::Point_3                                       Point_3;
typedef Kernel::Plane_3                                       Plane_3;
typedef CGAL::Polyhedron_3<Kernel>                            Polyhedron_3;
typedef Polyhedron_3::Vertex_handle                           Vertex_handle;
typedef Polyhedron_3::Facet                                   Facet;
typedef Polyhedron_3::Halfedge                                Halfedge;
typedef Polyhedron_3::Vertex_const_iterator                   Vertex_const_iterator;
typedef Polyhedron_3::Vertex_iterator                         Vertex_iterator;
typedef Polyhedron_3::Facet_const_iterator                    Facet_const_iterator;
typedef Polyhedron_3::Facet_iterator                          Facet_iterator;
typedef Polyhedron_3::Halfedge_const_iterator                 Halfedge_const_iterator;
typedef Polyhedron_3::Halfedge_iterator                       Halfedge_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator        Halfedge_around_facet_circulator;
typedef Polyhedron_3::Halfedge_handle                         Halfedge_handle;

typedef CGAL::Nef_polyhedron_3<Kernel, CGAL::SNC_indexed_items> Nef_polyhedron;
typedef Nef_polyhedron::Halfedge                                Nef_halfedge;
typedef Nef_polyhedron::Vertex_const_iterator                   Nef_vertex_const_iterator;
typedef Nef_polyhedron::Vertex_iterator                         Nef_vertex_iterator;
typedef Nef_polyhedron::Halfedge_const_iterator                 Nef_halfedge_const_iterator;
typedef Nef_polyhedron::Halfedge_iterator                       Nef_halfedge_iterator;
//typedef Nef_polyhedron::Volume_const_iterator Volume_const_iterator;

namespace cover{
  class OpenSetConvex: public OpenSet{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds);
      virtual ~OpenSetConvex(){};

      bool IsInside(ob::State *sPrime);

      //set computations
      virtual bool IsSubsetOf(const cover::OpenSet *rhs_, double tolerance = 1e-5) const override;
      const Polyhedron_3& GetPolyhedron() const;
      const Nef_polyhedron& GetNefPolyhedron() const;
      const Eigen::MatrixXd GetA() const;
      const Eigen::VectorXd GetB() const;
      std::vector<Eigen::VectorXd> vrep() const;
      bool contains(const Eigen::VectorXd& point, double tolerance) const;

      //O = O - O \cup rhs
      void RemoveIntersection(const cover::OpenSet *rhs_);

      //facet as ordered sequence of vertices around its outer edge. 
      //The orientation is given by the right-hand rule
      std::vector<Vector3> GetFacet(uint k); 
      uint GetNumberOfFacets();
      uint GetNumberOfInactiveFacets();
      bool IsActiveFacet(uint k);
      Vector3 GetCenterOfFacet(uint k);
      Vector3 GetRandomPointOnFacet(uint k);

      void DrawGL(GUIState&) override;
      void DrawGLPolyhedron(GUIState& state);
      void DrawGLEllipsoid(GUIState& state);
      void DrawGLNefPolyhedron(GUIState& state);
      virtual std::ostream& Print(std::ostream& out) const override;

    protected:
      iris::IRISRegion region;
      Polyhedron_3 poly; //inflated region of free workspace
      Nef_polyhedron nef_poly;
      Eigen::MatrixXd A_poly;
      Eigen::VectorXd b_poly;
      Polyhedron_3 poly_bounds; //bounding box region around workspace
      Eigen::MatrixXd A_bounds;
      Eigen::VectorXd b_bounds;
      ompl::RNG rng_;
  };
};
