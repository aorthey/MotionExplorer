#pragma once

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "elements/simplicial_complex.h"

namespace ob = ompl::base;
using PlannerDataGraphUndirected =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                          boost::property<vertex_type_t, ompl::base::PlannerDataVertex *,
                                          boost::property<boost::vertex_index_t, unsigned int>>,
                          boost::property<edge_type_t, ompl::base::PlannerDataEdge *,
                                          boost::property<boost::edge_weight_t, ompl::base::Cost>>>;

namespace Topology{
  class TopologicalGraph{
    public:
      TopologicalGraph(ob::PlannerData& pd, const ob::OptimizationObjective& obj);

      SimplicialComplex& GetSimplicialComplex();

      void ComputeShortestPaths(ob::PlannerData& pd, const ob::OptimizationObjective& opt);

    private:
      SimplicialComplex cmplx;
  };
}

