#pragma once

#include "planner/strategy/quotientgraph/quotient_graph.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QuotientSubGraph: public og::QuotientGraph{

        typedef og::QuotientGraph BaseT;
      public:
        struct GraphBundle{
          std::string name{"quotient_subgraph"};
        };
        typedef boost::subgraph<
          boost::adjacency_list<
           boost::vecS, 
           boost::vecS, 
           boost::undirectedS,
           Configuration*,
           boost::property<boost::edge_index_t, int, EdgeInternalState>
          >
         >SubGraph;

        typedef boost::graph_traits<SubGraph> BGT;
        typedef BGT::vertex_descriptor Vertex;
        typedef BGT::edge_descriptor Edge;
        typedef BGT::vertices_size_type VertexIndex;
        typedef BGT::in_edge_iterator IEIterator;
        typedef BGT::out_edge_iterator OEIterator;
        typedef Vertex* VertexParent;
        typedef VertexIndex* VertexRank;

      public:

        QuotientSubGraph(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
        ~QuotientSubGraph();
        void Rewire(Vertex &v);
        void Rewire();

        virtual void Grow(double t) = 0;

        virtual void DeleteConfiguration(Configuration *q) override;
        virtual Vertex AddConfiguration(Configuration *q) override;

    protected:

        SubGraph graphSparse_;
        SubGraph graphDense_;
        RoadmapNeighborsPtr nearestSparse_;
        RoadmapNeighborsPtr nearestDense_;

    };
  };
};


