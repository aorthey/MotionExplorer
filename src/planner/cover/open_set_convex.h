#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <iris/iris.h>
namespace ob = ompl::base;

class CSpaceOMPL;
// class Polyhedron_3;
// class Point_3;

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

namespace cover{
  class OpenSetConvex: public OpenSet{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds);
      virtual ~OpenSetConvex(){};
      bool IsInside(ob::State *sPrime);


      //facet as ordered sequence of vertices around its outer edge. 
      //The orientation is given by the right-hand rule
      std::vector<Vector3> GetFacet(uint k); 
      uint GetNumberOfFacets();
      bool IsActiveFacet(uint k);
      Vector3 GetCenterOfFacet(uint k);

      void DrawGL(GUIState&) override;

      virtual std::ostream& Print(std::ostream& out) const override;

    protected:
      iris::IRISRegion region;
      Polyhedron_3 poly; //inflated region of free workspace
      Polyhedron_3 poly_bounds; //bounding box region around workspace
      Eigen::MatrixXd A_bounds;
      Eigen::VectorXd b_bounds;
  };
};
