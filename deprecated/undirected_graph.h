#pragma once
#include "planner/strategy/ompl/quotient.h"
#include "planner/cover/open_set.h"
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/property_map/property_map.hpp>
using Math::dInf;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {

    template <typename VertexInternalState, typename EdgeInternalState>
    class UndirectedGraph{
      private:
        typedef unsigned long int VertexIndexType;
        typedef boost::adjacency_list<
           boost::vecS, 
           boost::vecS, 
           boost::undirectedS,
           VertexInternalState,
           EdgeInternalState
         >InternalUndirectedGraph;

        InternalUndirectedGraph G;

        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;
        typedef Vertex* VertexParent;
        typedef VertexIndex* VertexRank;

        typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
        typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

        std::map<Vertex, VertexRank> rank;
        std::map<Vertex, Vertex> parent;
        boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
          disjointSets_{boost::make_assoc_property_map(rank), boost::make_assoc_property_map(parent)};
      public:
        UndirectedGraph(){};
    };
  };
};
