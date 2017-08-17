#pragma once

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include "elements/simplicial_complex.h"

namespace ob = ompl::base;
namespace Topology{
  class TopologicalGraph{
    public:
      TopologicalGraph(ob::PlannerData& pd);

      SimplicialComplex& GetSimplicialComplex();

    private:
      SimplicialComplex cmplx;
  };
}

