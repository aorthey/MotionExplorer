#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/foreach.hpp>
#include <iostream>

struct VertexInternalState
{
  static int idx_glob;
  int index{idx_glob++};
  std::string name;
}; 

int VertexInternalState::idx_glob=0;

struct EdgeInternalState
{
  int index;
  double weight;
  std::string name;
};

typedef boost::adjacency_list<
  boost::vecS, 
  boost::vecS, 
  boost::undirectedS,
  VertexInternalState,
  EdgeInternalState
> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

std::ostream& operator<< (std::ostream& out, const Graph& G) 
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
      << " (" << G[e].index << ", " << G[e].weight << ")" << std::endl;
  }
  out << std::string(80, '-') << std::endl;
  out << "vertices: " << boost::num_vertices(G) 
    << " edges: " << boost::num_edges(G) << std::endl;
  return out;
}

int main(int argc,const char** argv)
{
  using namespace boost;

  Graph G;

  //Two ways of inserting vertices
  VertexInternalState u;
  u.name = "A";
  add_vertex(u, G);

  const Vertex v = add_vertex(G);
  G[v].name = "B";

  std::cout << G << std::endl;
  return 0;
}
