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

      std::vector<Math3D::Vector3> vertexIndicesToVector(const ob::PlannerData& pd, const std::vector<ob::PlannerData::Graph::Vertex> &v);
      Math3D::Vector3 vertexIndexToVector(const ob::PlannerData& pd, const ob::PlannerData::Graph::Vertex &v);

      std::vector<ob::PlannerData::Graph::Vertex> shortestPath(const ob::PlannerData::Graph::Vertex, const ob::PlannerData::Graph::Vertex, const ob::PlannerData::Graph::Vertex, const std::vector<ob::PlannerData::Graph::Vertex>&, const std::vector<ob::PlannerData::Graph::Vertex> &);

      bool testVisibilityRRT(const ob::PlannerData& pd, const ob::SpaceInformationPtr &si_path_space, const std::vector<ob::PlannerData::Graph::Vertex> &p1, const std::vector<ob::PlannerData::Graph::Vertex> &p2);

      template<typename T>
      std::vector<std::vector<T> > extractFacetsBetweenPaths( const std::vector<T> &p1, const std::vector<T> &p2);

    private:
      SimplicialComplex cmplx;
  };
}

