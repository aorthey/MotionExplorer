#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/foreach.hpp>

struct VertexInternalState
{
  VertexInternalState(int idx): index(idx){};
  int index;
  std::string name;
}; 

struct EdgeInternalState
{
  bool visted;
  double weight;
  std::string name;
};

//not a valid subgraph: does not have edge index
typedef boost::adjacency_list<
  boost::vecS, 
  boost::vecS, 
  boost::undirectedS,
  VertexInternalState,
  EdgeInternalState
> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

int main(int argc,const char** argv)
{
  using namespace boost;

  Graph G;
  VertexInternalState u(0);
  u.name = "A";
  add_vertex(u, G);

  return 0;
}
