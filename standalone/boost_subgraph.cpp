#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>

//
//Working Example of two overlapping bundled subgraphs
//
// The graphs created look like
//
//G0:                                         |
//    A -- B -- C                             |
//   /           \                            |
// xI             xG                          |
//  \            /                            |
//   E -------- F                             |
//                                            |
//G1 \subset G0:                              |
//                                            |
//    A -- B -- C                             |
//   /           \                            |
// xI             xG                          |
//                                            |
//G2 \subset G0:                              |
//                                            |
// xI             xG                          |
//  \            /                            |
//   E -------- F                             |

struct VertexInternalState
{
  int index{0};
  std::string name;
}; 

struct EdgeInternalState
{
  int index{0};
  bool visted{false};
  double weight{1.0};
  std::string name;
  double getCost(){
    return weight;
  }
};
struct GraphBundle{
  std::string name{"boost_subgraph"};
};

using containerS = boost::vecS;
typedef boost::subgraph<
  boost::adjacency_list<
    containerS,
    containerS, 
    boost::undirectedS,
    boost::property<boost::vertex_index_t, int, VertexInternalState>,
    boost::property<boost::edge_index_t, int, EdgeInternalState>,
    GraphBundle
    >
>SubGraph;

typedef boost::graph_traits<SubGraph>::vertex_descriptor Vertex;
typedef boost::graph_traits<SubGraph>::edge_descriptor Edge;

std::ostream& operator<< (std::ostream& out, const SubGraph& G) 
{
  out << std::string(80, '-') << std::endl;
  //out << "Graph " << G[boost::graph_bundle] << std::endl; 

  BOOST_FOREACH(const Vertex v, boost::vertices(G))
  {
    out << "vertex: idx " << G[v].index << " name " << G[v].name << std::endl;
  }
  BOOST_FOREACH(const Edge e, boost::edges(G))
  {
    const Vertex u = boost::source(e,G);
    const Vertex v = boost::target(e,G);
    out << "edge " << G[u].name << "-" << G[v].name 
      << " (" << G[e].index << ", " << G[e].weight << ")" << std::endl;
  }
  out << std::string(80, '-') << std::endl;
  out << "vertices: " << boost::num_vertices(G) 
    << " edges: " << boost::num_edges(G) << std::endl;
  return out;
}

Vertex AddVertex(SubGraph &G, std::string name)
{
  static int idx = 0;
  Vertex v = add_vertex(G);
  G[v].index = idx++;
  G[v].name = name;
  return v;
}
Edge AddEdge(SubGraph &G, const Vertex v, const Vertex w, std::string name = "")
{
  static int idx = 0;
  std::pair<Edge, bool> result = add_edge(v, w, G);
  G[result.first].name = name;
  G[result.first].index = idx++;
  return result.first;
}
int main(int argc,const char** argv)
{
  SubGraph G0;
  SubGraph& G1 = G0.create_subgraph();
  SubGraph& G2 = G0.create_subgraph();
  // G0[boost::graph_bundle].name = "G0";
  // G1[boost::graph_bundle].name = "G1";
  // G2[boost::graph_bundle].name = "G2";

  //double insertion into G1,G2
  const Vertex vI = AddVertex(G1, "xI");
  add_vertex(vI, G2);
  const Vertex vG = AddVertex(G1, "xG");
  add_vertex(vG, G2);


  //single insertion into G1
  const Vertex v1 = AddVertex(G1, "A");
  const Vertex v2 = AddVertex(G1, "B");
  const Vertex v3 = AddVertex(G1, "C");
  AddEdge(G1, vI, v1);
  AddEdge(G1, v1, v2);
  AddEdge(G1, v2, v3);
  AddEdge(G1, v3, vG);

  //single insertion into G2
  const Vertex v4 = AddVertex(G2, "E");
  const Vertex v5 = AddVertex(G2, "F");
  AddEdge(G2, vI, v4);
  AddEdge(G2, v4, v5);
  AddEdge(G2, v5, vG);

  //insert edge into G1 and G2 (works because both vertices are each in G1 and
  //G2) --- but we should probably not do it, because can never be sure that it
  //will insert an edge into every graph
  AddEdge(G0, vI, vG);

  std::cout << G0 << std::endl;
  std::cout << G1 << std::endl;
  std::cout << G2 << std::endl;

  return 0;
}
