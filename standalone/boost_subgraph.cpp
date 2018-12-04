#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/foreach.hpp>
#include <iostream>

/// adventures using bundled properties with subgraphs. works only for vertices
//so far. Tried to apply patches #10709 and #10708 from boost bugtracking:
//https://svn.boost.org/trac10/attachment/ticket/10708/
//https://svn.boost.org/trac10/attachment/ticket/10709/
//
// #10709: could not apply completely, because it says overload not allowed (?)
// #10708: partial application, needs some change in function add_vertex(), but
// function is not virtual

namespace boost{
  template <typename Graph>
  class bundled_subgraph: public boost::subgraph<Graph>{

    public:
      typename graph::detail::bundled_result<Graph, graph_bundle_t>::type const&
      operator[](graph_bundle_t x) const
      { return boost::subgraph<Graph>::m_graph[x]; }

      // ORIGINAL PATCH #10709
      // typename graph::detail::bundled_result<Graph, graph_bundle_t>::type&
      // operator[](graph_bundle_t x) const
      // { return m_graph[x]; }
      // typename graph::detail::bundled_result<Graph, graph_bundle_t>::type const&
      // operator[](graph_bundle_t x) const
      // { return m_graph[x]; }
  };
  namespace detail {
    template <typename G>
    typename subgraph<G>::vertex_descriptor
    add_vertex_recur_up(const typename G::vertex_property_type& p, 
          subgraph<G>& g)
    {
        typename subgraph<G>::vertex_descriptor u_local, u_global;
        if (g.is_root()) {
            //u_global = add_vertex(g.m_graph);
            u_global = add_vertex(p, g.m_graph);
            g.m_global_vertex.push_back(u_global);
        } else {
            //u_global = add_vertex_recur_up(*g.m_parent);
            u_global = add_vertex_recur_up(p, *g.m_parent);
            u_local = add_vertex(g.m_graph);
            g.m_global_vertex.push_back(u_global);
            g.m_local_vertex[u_global] = u_local;
        }
        return u_global;
    }
  } // namespace detail

  template <typename G>
  typename subgraph<G>::vertex_descriptor
  add_vertex(const typename G::vertex_property_type& p, subgraph<G>& g)
  {
    typename subgraph<G>::vertex_descriptor  u_local, u_global;
    u_global = detail::add_vertex_recur_up(p, g);
    u_local = g.global_to_local(u_global);
    return u_local;
  }
};


struct VertexInternalState
{
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
> GraphA;

//valid
typedef boost::adjacency_list<
  boost::vecS, 
  boost::vecS, 
  boost::undirectedS, 
  VertexInternalState, 
  boost::property<boost::edge_index_t,int, EdgeInternalState> 
> GraphB;

//valid
typedef boost::adjacency_list<
  boost::setS, 
  boost::setS, 
  boost::directedS, 
  VertexInternalState, 
  boost::property<boost::edge_index_t,int, EdgeInternalState> 
> GraphC;

//#############################################################################
typedef GraphB Graph;
//#############################################################################

typedef boost::bundled_subgraph<Graph> SubGraph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
//typedef BGT::vertices_size_type VertexIndex;
//typedef BGT::in_edge_iterator IEIterator;
//typedef BGT::out_edge_iterator OEIterator;

void PrintGraph(const SubGraph &G)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Graph " << std::endl;
  BOOST_FOREACH(const Vertex v, boost::vertices(G))
  {
    VertexInternalState vi = get(boost::vertex_bundle, G)[v];
    std::cout << "vertex " << vi.index << " " << vi.name << std::endl;
  }
  // BOOST_FOREACH(const Edge e, boost::edges(G))
  // {
  //   const Vertex u = boost::source(e,G);
  //   const Vertex v = boost::target(e,G);
  //   std::cout << "edge " << G[u].name << "-" << G[v].name << " (" << G[e].name << ")" << std::endl;
  // }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "vertices: " << boost::num_vertices(G) << " edges: " << boost::num_edges(G) << std::endl;
}

int main(int argc,const char** argv)
{
  using namespace boost;

  SubGraph G;
  VertexInternalState u;
  u.index = 0;
  u.name = "A";
  VertexInternalState v;
  v.index = 0;
  v.name = "B";
  VertexInternalState w;
  w.index = 0;
  w.name = "C";
  add_vertex(u, G);
  add_vertex(v, G);
  add_vertex(w, G);

  //// Vertex w = add_vertex(G);
  //// G[w].index = 2;
  //// G[w].name = "C";

  // EdgeInternalState e1;
  // e1.weight = 1.0;
  // e1.visted = false;
  // e1.name = "MainStreet";
  // add_edge(u, v, e1, G);

  //// EdgeInternalState e2;
  //// e2.weight = 1.0;
  //// e2.visted = false;
  //// e2.name = "SecondStreet";
  //// add_edge(v, w, e2, G);

  ////PrintGraph(G);

  ////Graph& G1 = G.create_subgraph();
  PrintGraph(G);
  return 0;
}
