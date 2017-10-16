#include "lemon_interface.h"

using namespace lemon;
LemonInterface( ob::PlannerDataPtr pd_ ):
  pd(pd_)
{
//#############################################################################
// Nodes boost to lemon
//#############################################################################

  const ob::SpaceInformationPtr si = pd->getSpaceInformation();
  N = pd->numVertices();

  //ListGraph lg;
  //std::vector<ListGraph::Node> gn;
  //ListGraph::Node start, goal;

  for(uint k = 0; k < N; k++){
    ListGraph::Node x = lg.addNode();
    gn.push_back(x);
    if(pd->isStartVertex(k)) start = x;
    if(pd->isGoalVertex(k)) goal = x;
  }

//#############################################################################
// Extract edges and weight
//#############################################################################
  //typedef ListGraph::EdgeMap<double> CostMap;
  //CostMap length(lg);
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
      length[e] = c.value();
    }
  }

}
bool IsConnected(){
  //auto dstart = Dijkstra<ListGraph, CostMap>(lg, length);
  //dstart.run(start);
  //auto dgoal = Dijkstra<ListGraph, CostMap>(lg, length);
  //dgoal.run(goal);
  auto dijkstra = Dijkstra<ListGraph, CostMap>(lg, length);
  //bool reached = dijkstra.path(p).dist(d).run(s,t);
  bool reached = dijkstra.run(start,goal);
  return reached;
}
         // Compute shortest path from node s to each node
         //        dijkstra(g,length).predMap(preds).distMap(dists).run(s);
         //             
         //                    // Compute shortest path from s to t
         //                           bool reached =
         //                           dijkstra(g,length).path(p).dist(d).run(s,t);

