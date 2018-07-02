#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <iris/iris.h>
#include <ompl/util/RandomNumbers.h>
#include "elements/geometry/convex_polyhedron.h"
#include "elements/geometry/nef_polyhedron.h"
#include "elements/geometry/ellipsoid.h"
namespace ob = ompl::base;

class CSpaceOMPL;

namespace cover{
  class OpenSetConvex: public OpenSet{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds);
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd center);
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, ConvexPolyhedron *poly_);
      virtual ~OpenSetConvex(){};

      virtual bool IsSubsetOf(const cover::OpenSet *rhs_, double tolerance = 1e-5) const override;
      virtual bool IsInside(ob::State *sPrime) override;

      //facet as ordered sequence of vertices around its outer edge. 
      //The orientation is given by the right-hand rule
      // std::vector<Vector3> GetFacet(uint k); 
      // uint GetNumberOfFacets();
      // uint GetNumberOfInactiveFacets();
      // bool IsActiveFacet(uint k);
      // Vector3 GetCenterOfFacet(uint k);
      // Vector3 GetRandomPointOnFacet(uint k);

      void RandomState(ob::State *s);

      virtual void DrawGL(GUIState&) override;
      virtual std::ostream& Print(std::ostream& out) const override;

    protected:
      iris::IRISRegion region;
      Ellipsoid *ellipsoid{nullptr};
      ConvexPolyhedron *polyhedron{nullptr};
      ConvexPolyhedron *polyhedron_bounds{nullptr};
      NefPolyhedron *nef_polyhedron{nullptr};
      std::vector<ConvexPolyhedron> cvx_decomposition;

  };
};
