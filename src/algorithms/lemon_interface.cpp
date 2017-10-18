#include "lemon_interface.h"

using namespace lemon;
using Cost = ob::Cost;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;
using Edge = Graph::Edge;
using VIterator = Graph::VIterator;
using EIterator = Graph::EIterator;

LemonInterface::LemonInterface( ob::PlannerDataPtr pd_ ):
  pd(pd_)
{
//#############################################################################
// Nodes boost to lemon
//#############################################################################

  const ob::SpaceInformationPtr si = pd->getSpaceInformation();
  N = pd->numVertices();

  for(uint k = 0; k < N; k++){
    ListGraph::Node x = lg.addNode();
    gn.push_back(x);
    if(pd->isStartVertex(k)) start = x;
    if(pd->isGoalVertex(k)) goal = x;
  }

//#############################################################################
// Extract edges and weight
//#############################################################################
  length = new CostMap(lg);

  for(uint v1 = 0; v1 < pd->numVertices(); v1++){
    ob::PlannerDataVertex v1d = pd->getVertex(v1);

    std::map<unsigned int, const ob::PlannerDataEdge *> edgeMap;
    pd->getEdges(v1, edgeMap);

    for (const auto &e1 : edgeMap) {
      uint v2 = e1.first;

      ob::Cost c;
      pd->getEdgeWeight(v1,v2,&c);

      ListGraph::Edge e = lg.addEdge( gn.at(v1), gn.at(v2) );
      (*length)[e] = c.value();
    }
  }

}
std::vector<Vertex> LemonInterface::GetShortestPath(){
  return GetShortestPath(start, goal);
}
std::vector<Vertex> LemonInterface::GetShortestPath( ListGraph::Node s, ListGraph::Node t){

  auto dijkstra = Dijkstra<ListGraph, CostMap>(lg, *length);
  bool reached = dijkstra.run(s,t);

  if(reached){
    Path<ListGraph> path_start = dijkstra.path(start);
    std::vector<ListGraph::Node> path;

    for (Path<ListGraph>::ArcIt it(path_start); it != INVALID; ++it) {
      ListGraph::Node v = lg.source(it);
      ListGraph::Node w = lg.target(it);
      path.push_back(v);
      path.push_back(w);
    }

    if(path.size()>0){
      double L = 0.0;
      for(uint k = 0; k < path.size()-1; k++){
        ListGraph::Node v = path.at(k);
        ListGraph::Node w = path.at(k+1);
        uint v1i = lg.id(v);
        uint v2i = lg.id(w);
        if(v1i!=v2i){
          //const ob::State *s1 = pd->getVertex(v1i).getState();
          //const ob::State *s2 = pd->getVertex(v2i).getState();
          shortest_path_idxs.push_back(v1i);
          shortest_path_idxs.push_back(v2i);
        }
      }
    }

  }
  return shortest_path_idxs;

}

