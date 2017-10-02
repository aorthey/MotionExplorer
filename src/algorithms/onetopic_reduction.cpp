#include "algorithms/onetopic_reduction.h"
#include "algorithms/path_visibility.h"
#include <ompl/util/Console.h>

OnetopicPathSpaceModifier::OnetopicPathSpaceModifier( ob::PlannerData& pd_in, CSpaceOMPL *cspace_ ):
cspace(cspace_)
{
  ComputeShortestPathsLemon(pd_in);
  InterpolatePaths(pd_in);
}

void OnetopicPathSpaceModifier::ComputeShortestPathsLemon(ob::PlannerData& pd_in){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Onetopic Reduction" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "input paths    : " << pd_in.numVertices() << std::endl;
  std::cout << "output paths   : " << std::flush;

//#############################################################################
// output
//#############################################################################

  std::vector<Config> V;
  std::vector<int> Vidx;
  std::vector< std::vector<ob::PlannerData::Graph::Vertex> > V_shortest_path;
  std::vector<double> distance_shortest_path; //length of the shortest path from start to goal >including< vertex i
  double max_distance_shortest_path;
  double min_distance_shortest_path;

//#############################################################################
// Nodes boost to lemon
//#############################################################################
  using namespace lemon;

  const ob::SpaceInformationPtr si = pd_in.getSpaceInformation();
  Graph g = pd_in.toBoostGraph();
  uint N = pd_in.numVertices();

  ListGraph lg;
  std::vector<ListGraph::Node> gn;
  ListGraph::Node start, goal;

  for(uint k = 0; k < N; k++){
    ListGraph::Node x = lg.addNode();
    gn.push_back(x);
    if(pd_in.isStartVertex(k)) start = x;
    if(pd_in.isGoalVertex(k)) goal = x;
  }

//#############################################################################
// Extract edges and weight
//#############################################################################
  typedef ListGraph::EdgeMap<double> CostMap;
  CostMap length(lg);

  for(uint v1 = 0; v1 < pd_in.numVertices(); v1++){
    ob::PlannerDataVertex v1d = pd_in.getVertex(v1);

    std::map<unsigned int, const ob::PlannerDataEdge *> edgeMap;
    pd_in.getEdges(v1, edgeMap);

    for (const auto &e1 : edgeMap) {
      uint v2 = e1.first;

      ob::Cost c;
      pd_in.getEdgeWeight(v1,v2,&c);

      ListGraph::Edge e = lg.addEdge( gn.at(v1), gn.at(v2) );
      length[e] = c.value();
    }
  }

//#############################################################################
// Dijkstra on graph
//#############################################################################

  auto dstart = Dijkstra<ListGraph, CostMap>(lg, length);
  dstart.run(start);

  auto dgoal = Dijkstra<ListGraph, CostMap>(lg, length);
  dgoal.run(goal);

  V_shortest_path.resize(N);
  V.resize(N);
  Vidx.resize(N);
  distance_shortest_path.resize(N);

  for (ListGraph::NodeIt node(lg); node != INVALID; ++node)
  {
    uint node_idx = lg.id(node);

    Path<ListGraph> path_start = dstart.path(node);
    std::vector<ListGraph::Node> path;

    for (Path<ListGraph>::ArcIt it(path_start); it != INVALID; ++it) {
      ListGraph::Node v = lg.source(it);
      ListGraph::Node w = lg.target(it);
      path.push_back(v);
      path.push_back(w);
    }
    Path<ListGraph> path_goal = dgoal.path(node);
    std::vector<ListGraph::Node> path_reversed;
    for (Path<ListGraph>::ArcIt it(path_goal); it != INVALID; ++it) {
      ListGraph::Node v = lg.source(it);
      ListGraph::Node w = lg.target(it);
      path_reversed.push_back(v);
      path_reversed.push_back(w);
    }
    path.insert( path.end(), path_reversed.rbegin(), path_reversed.rend() );

    if(path.size()>0){
      double L = 0.0;
      std::vector<Vertex> shortest_path_idxs;
      for(uint k = 0; k < path.size()-1; k++){
        ListGraph::Node v = path.at(k);
        ListGraph::Node w = path.at(k+1);
        uint v1i = lg.id(v);
        uint v2i = lg.id(w);
        if(v1i!=v2i){
          const ob::State *s1 = pd_in.getVertex(v1i).getState();
          const ob::State *s2 = pd_in.getVertex(v2i).getState();
          Config q1 = cspace->OMPLStateToConfig(s1);
          Config q2 = cspace->OMPLStateToConfig(s2);
          L += (q1-q2).norm();

          shortest_path_idxs.push_back(v1i);
          shortest_path_idxs.push_back(v2i);
        }
      }
      const ob::State *sn = pd_in.getVertex(node_idx).getState();
      V.at(node_idx) = cspace->OMPLStateToConfig(sn);
      V_shortest_path.at(node_idx) = shortest_path_idxs;
      distance_shortest_path.at(node_idx) = L;

    }else{
      V.at(node_idx).setZero();
      distance_shortest_path.at(node_idx) = dInf;
    }


  }
  if(distance_shortest_path.size()>0){
    uint idx_min = std::distance(std::begin(distance_shortest_path), std::min_element(std::begin(distance_shortest_path), std::end(distance_shortest_path)));
    uint idx_max = std::distance(std::begin(distance_shortest_path), std::max_element(std::begin(distance_shortest_path), std::end(distance_shortest_path)));
    min_distance_shortest_path = distance_shortest_path[idx_min];
    max_distance_shortest_path = distance_shortest_path[idx_max];
    std::cout << "min shortest path: " << min_distance_shortest_path << std::endl;
    std::cout << "max shortest path: " << max_distance_shortest_path << std::endl;
  }

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  typedef boost::graph_traits < PlannerDataGraph >::adjacency_iterator adjacency_iterator;
  

  PathVisibilityChecker path_checker(si, cspace);
  for(uint i = 0; i < Vidx.size(); i++){
   
    double lk = distance_shortest_path.at(i);
    if(lk>=dInf) continue;
    Vertex v_current = i;

    std::pair<adjacency_iterator, adjacency_iterator> neighbors =
      boost::adjacent_vertices(vertex(v_current,g), g);
   
    bool neighborIsBetter = false;
    for(; neighbors.first != neighbors.second; ++neighbors.first){
      Vertex v_neighbor = *neighbors.first;
      double ln = distance_shortest_path.at(v_neighbor);
      if( ln < (lk - 1e-3)){
        neighborIsBetter = true;
        //found better neighbor

        //extract shortest paths and compare their visibility:
        const std::vector<Vertex> pcur_idxs = V_shortest_path.at(v_current);
        const std::vector<Vertex> pneighbor_idxs = V_shortest_path.at(v_neighbor);

        neighborIsBetter = path_checker.isVisible(pd_in, pcur_idxs, pneighbor_idxs);

        if(neighborIsBetter) break; //don't check the other neighbors, we found at least one
      }
    }
    if(!neighborIsBetter){
      std::vector<Vertex> Vidxs = V_shortest_path.at(i);
      if(Vidxs.size()>0){
        std::vector<const ob::State*> omplstate_path;
        for(uint k = 0; k < Vidxs.size(); k++){
          const ob::State *sk = pd_in.getVertex(Vidxs.at(k)).getState();
          omplstate_path.push_back(sk);
        }
        omplstate_paths.push_back(omplstate_path);
      }
    }
  }
  std::cout << omplstate_paths.size() << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

void OnetopicPathSpaceModifier::InterpolatePaths( ob::PlannerData& pd ){
  const ob::SpaceInformationPtr si = pd.getSpaceInformation();

  config_paths.clear();
  for(uint i = 0; i < omplstate_paths.size(); i++){
    std::vector<const ob::State*> states = omplstate_paths.at(i);
    og::PathGeometric path(si);
    for(uint k = 0; k < states.size(); k++){
      path.append(states.at(k));
    }

    og::PathSimplifier shortcutter(si);
    shortcutter.shortcutPath(path);

    path.interpolate();

    std::vector<ob::State *> interpolated_states = path.getStates();
    std::vector<Config> keyframes;
    for(uint k = 0; k < interpolated_states.size(); k++)
    {
      ob::State *state = interpolated_states.at(k);
      Config q = cspace->OMPLStateToConfig(state);
      keyframes.push_back(q);
    }
    config_paths.push_back(keyframes);
  }
}

std::vector< std::vector< Config >> OnetopicPathSpaceModifier::GetConfigPaths(){
  return config_paths;
}
std::vector< std::vector< const ob::State* >> OnetopicPathSpaceModifier::GetOMPLStatePaths(){
  return omplstate_paths;
}
