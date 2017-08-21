#pragma once

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "elements/simplicial_complex.h"

namespace ob = ompl::base;
namespace Topology{
  class TopologicalGraph{
    public:
      TopologicalGraph(const ob::PlannerData& pd, const ob::OptimizationObjective& obj);

      SimplicialComplex& GetSimplicialComplex();

      //void ComputeShortestPaths(const ob::PlannerData& pd);
      void ComputeShortestPaths(const ob::PlannerData& pd, const ob::OptimizationObjective& opt);

    private:
      SimplicialComplex cmplx;
  };
}

