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
    // VertexInternalState,
    boost::property<boost::edge_index_t, int, EdgeInternalState>
    //Graphbundle currently NOT supported by boost::subgraph
    >
>SubGraph;

typedef boost::graph_traits<SubGraph>::vertex_descriptor Vertex;
typedef boost::graph_traits<SubGraph>::edge_descriptor Edge;

std::ostream& operator<< (std::ostream& out, const SubGraph& G) 
{
  out << std::string(80, '-') << std::endl;
  out << "Graph " << std::endl; 

  BOOST_FOREACH(const Vertex v, boost::vertices(G))
  {
    out << "vertex: idx " << G[v].index << " name " << G[v].name << std::endl;
  }
  BOOST_FOREACH(const Edge e, boost::edges(G))
  {
    const Vertex u = boost::source(e,G);
    const Vertex v = boost::target(e,G);
    out << "edge " << G[u].name << "-" << G[v].name 
      << " (idx=" << G[e].index << ", weight=" << G[e].weight << ")" << std::endl;
  }
  if(boost::num_vertices(G) > 0){
    out << std::string(80, '-') << std::endl;
    out << "vertices: " << boost::num_vertices(G) 
    << " edges: " << boost::num_edges(G);
  }else{
    out << "Empty Graph";
  }
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
void DeleteVertex(SubGraph &G, Vertex v)
{
  //need to go manual, remove_vertex NYI in boost 1.70

  while (true) {
    typedef typename SubGraph::out_edge_iterator oei_type;
    std::pair<oei_type, oei_type> p = out_edges(v, G);
    if (p.first == p.second) break;
    remove_edge(*p.first,  G.m_graph);
  }
  std::cout << "Delete " << G[v].name << std::endl;
  remove_vertex(v, G.m_graph);

  //need to remake the idx's
}

Edge AddEdge(SubGraph &G, const Vertex v, const Vertex w, std::string name = "")
{
  static int idx = 0;
  std::pair<Edge, bool> result = add_edge(v, w, G);
  G[result.first].name = name;
  G[result.first].index = idx++;
  return result.first;
}


//(1) If we add two vertices to a subgraph, and there exists an edge on the
//parent graph, then this edge is automatically added to the subgraph
int main(int argc,const char** argv)
{
  SubGraph G0;
  SubGraph& G1 = G0.create_subgraph();
  //   G0                           |
  //  /                             |
  // G1

  //(1) Insert into G1 (inserts automatically into G0)
  const Vertex vI = AddVertex(G1, "xI");

  AddVertex(G0, "0");

  //(2) Let us add some subgraph structure to G1 (automatically added to G0)
  const Vertex v1 = AddVertex(G1, "A");
  const Vertex v2 = AddVertex(G1, "B");
  const Vertex v3 = AddVertex(G1, "C");
  const Vertex v4 = AddVertex(G1, "D");
  AddEdge(G1, vI, v1);
  AddEdge(G1, v1, v2);
  AddEdge(G1, v2, v3);
  AddEdge(G1, v3, v4);

  std::cout << G0 << std::endl;
  std::cout << G1 << std::endl;

  std::cout << std::string(80, '*') << std::endl;
  std::cout << "Delete vertex" << std::endl;
  std::cout << std::string(80, '*') << std::endl;
  DeleteVertex(G1, v2);

  std::cout << G0 << std::endl;
  std::cout << G1 << std::endl;
  return 0;
}
