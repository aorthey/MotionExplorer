#include "simplicial_complex_hasse_diagram.h"
#include "common.h"

using namespace ompl::geometric::topology;
void HasseDiagram::RemoveNode(SimplexNode s)
{
  // if( (a != S[sn].vertices.at(0) && a != S[sn].vertices.at(1))
  //     ||(b != S[sn].vertices.at(0) && b != S[sn].vertices.at(1)))
  // {
  //   std::cout << ">>> simplex representation differs." << std::endl;
  //   std::cout << ">>> edge " << a << "-" << b << std::endl;
  //   std::cout << ">>> simplex : " << S[sn].vertices << std::endl;
  //   exit(0);
  // }
  //std::cout << "removing edge " << a << "-" << b << ": associated simplex " << S[sn].vertices << std::endl;

  SC_OEIterator eo, eo_end, next;
  tie(eo, eo_end) = boost::out_edges(s, S);
  for (next = eo; eo != eo_end; eo = next) {
    ++next;
    SimplexNode sn = boost::target(*eo, S);
    RemoveNode(sn);
  }

  //remove simplex from k-simplices map
  uint N = S[s].vertices.size()-1;
  auto it = k_simplices.at(N).find(S[s].vertices);

  if(it!=k_simplices.at(N).end()){
    k_simplices.at(N).erase(it);
  }else{
    std::cout << "removing simplex " << S[s].vertices << std::endl;
    std::cout << "simplex " << S[s].vertices << " has not been found in map." << std::endl;
    std::cout << k_simplices.at(N) << std::endl;
    exit(0);
  }

  //remove facet ptrs
  boost::clear_vertex(s, S);
  //remove simplex from hasse diagram
  boost::remove_vertex(s, S);
}
void HasseDiagram::SetMaxDimension(uint max_dimension_)
{
  max_dimension = max_dimension_;
  k_simplices.resize(max_dimension+1);
}

HasseDiagram::SimplexNode HasseDiagram::AddNode(std::vector<vertex_t> v)
{
  std::sort(v.begin(), v.end());
  uint N = v.size()-1;
  auto it = k_simplices.at(N).find(v);
  if(it!=k_simplices.at(N).end()){
    return (*it).second;
  }else{
    SimplexNode s = boost::add_vertex(S);
    S[s].vertices = v;
    k_simplices.at(v.size()-1)[v] = s;
    if(v.size()>2) AddIncomingEdges(s);
    return s;
  }
}
void HasseDiagram::AddIncomingEdges(SimplexNode sigma)
{
  const std::vector<vertex_t>& vertices = S[sigma].vertices;
  uint N = vertices.size();
  uint N_facet_dimension = N-2;
  
  //iterate over all facets by taking all vertex permuatations of size N-1
  std::string bitmask(N-1, 1); // N-1 leading 1's
  bitmask.resize(N, 0); // 1 trailing 0

  do{
    std::vector<unsigned long int> facet;
    for (uint i = 0; i < N; i++)
    {
      if (bitmask[i]) facet.push_back(vertices.at(i));
    }
    auto it = k_simplices.at(N_facet_dimension).find(facet);
    SimplexNode tau;
    if(it == k_simplices.at(N_facet_dimension).end())
    {
      //std::cout << "tried adding coface " << vertices << " to facet " << facet << std::endl;
      //std::cout << "BUT: facet " << facet << " does not exist in simplex_map" << std::endl;
      //std::cout << "simplex map: " << k_simplices.at(N_facet_dimension) << std::endl;
      //exit(0);
      tau = AddNode(facet);
    }else{
      tau = (*it).second;
    }
    boost::add_edge(tau, sigma, S);
  }while(std::prev_permutation(bitmask.begin(), bitmask.end()));
}

const HasseDiagram::SimplicialComplexDiagram& HasseDiagram::GetDiagram()
{
  return S;
}
