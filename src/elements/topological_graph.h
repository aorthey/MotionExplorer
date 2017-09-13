#pragma once
#include "elements/simplicial_complex.h"

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>

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

      std::vector<Math3D::Vector3> vertexIndicesToVector(const ob::PlannerData& pd, std::vector<ob::PlannerData::Graph::Vertex> &v);
      Math3D::Vector3 vertexIndexToVector(const ob::PlannerData& pd, ob::PlannerData::Graph::Vertex v);

    private:
      SimplicialComplex cmplx;
  };
}

