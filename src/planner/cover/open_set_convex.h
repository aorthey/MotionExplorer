#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <iris/iris.h>
#include <ompl/util/RandomNumbers.h>
#include "elements/polyhedron/convex_polyhedron.h"
#include "elements/polyhedron/nef_polyhedron.h"
namespace ob = ompl::base;

class CSpaceOMPL;

namespace cover{
  class OpenSetConvex: public OpenSet{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_, const iris::Polyhedron &bounds);
      virtual ~OpenSetConvex(){};

      virtual bool IsSubsetOf(const cover::OpenSet *rhs_, double tolerance = 1e-5) const override;
      virtual bool IsInside(ob::State *sPrime) override;

      //void ComputeEnvironment();

      //facet as ordered sequence of vertices around its outer edge. 
      //The orientation is given by the right-hand rule
      // std::vector<Vector3> GetFacet(uint k); 
      // uint GetNumberOfFacets();
      // uint GetNumberOfInactiveFacets();
      // bool IsActiveFacet(uint k);
      // Vector3 GetCenterOfFacet(uint k);
      // Vector3 GetRandomPointOnFacet(uint k);

      virtual void DrawGL(GUIState&) override;
      //void DrawGLEllipsoid(GUIState& state);
      virtual std::ostream& Print(std::ostream& out) const override;

    protected:
      iris::IRISRegion region;
      ConvexPolyhedron *polyhedron;
      ConvexPolyhedron *polyhedron_bounds;
      NefPolyhedron *nef_polyhedron;
      std::vector<ConvexPolyhedron> cvx_decomposition;

  };
};
