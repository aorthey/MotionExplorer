#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

namespace ompl
{
  namespace geometric
  {
    namespace topology
    {
      class HasseDiagram{
        typedef long unsigned int vertex_t;

        protected:
          uint max_dimension{0};

          //###################################################################
          //Simplicial Complex as Hasse Diagram representation using a boost
          //directed graph
          //###################################################################

          class SimplexNodeInternalState{
            public:
              SimplexNodeInternalState() = default;
              SimplexNodeInternalState(const SimplexNodeInternalState &vis) = default;
              std::vector<vertex_t> vertices;
          };
          class SimplexConnectionInternalState{
            public:
              SimplexConnectionInternalState() = default;
          };

        public:
          typedef boost::adjacency_list<
             boost::setS,  //do not change to vecS, otherwise vertex indices are not stable after removal
             boost::setS, 
             boost::directedS,
             SimplexNodeInternalState,
             SimplexConnectionInternalState
           > SimplicialComplexDiagram;

          typedef boost::graph_traits<SimplicialComplexDiagram> SCBGT;
          typedef SCBGT::vertex_descriptor SimplexNode;
          typedef SCBGT::edge_descriptor SimplexEdge;
          typedef SCBGT::in_edge_iterator SC_IEIterator;
          typedef SCBGT::out_edge_iterator SC_OEIterator;

          HasseDiagram() = default;

          SimplexNode AddNode(std::vector<vertex_t>);
          void RemoveNode(SimplexNode);

          void SetMaxDimension(uint max_dimension_);

          const SimplicialComplexDiagram& GetDiagram();
          std::vector<std::map<std::vector<vertex_t>, SimplexNode>> k_simplices;
        protected:

          void AddIncomingEdges(SimplexNode sigma);
          SimplicialComplexDiagram S;

      };


    }
  }
}
