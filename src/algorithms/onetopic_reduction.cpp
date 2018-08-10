#include "algorithms/onetopic_reduction.h"
#include "algorithms/path_visibility.h"
#include <ompl/util/Console.h>

OnetopicPathSpaceModifier::OnetopicPathSpaceModifier( ob::PlannerDataPtr pd_in, CSpaceOMPL *cspace_ ):
pd(pd_in), cspace(cspace_)
{
  pd->decoupleFromPlanner();
  ComputeShortestPathsLemon();
  //ShortestPathsNonOnetopic();
  //ComputeOnetopicCoverRoadmap();
  //ComputeCoverVertices(pd_in);
}

/// output:
//   all_vertices
//   all_vertices_config
//   all_paths
//   all_paths_config
//   distance_shortest_path

void OnetopicPathSpaceModifier::ComputeShortestPathsLemon(){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Onetopic Reduction" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "input paths    : " << pd->numVertices() << std::endl;
  std::cout << "output paths   : " << std::flush;

//#############################################################################
// Nodes boost to lemon
//#############################################################################
  using namespace lemon;

  const ob::SpaceInformationPtr si = pd->getSpaceInformation();
  uint N = pd->numVertices();

  ListGraph lg;
  std::vector<ListGraph::Node> gn;
  ListGraph::Node start, goal;

  for(uint k = 0; k < N; k++){
    ListGraph::Node x = lg.addNode();
    gn.push_back(x);
    if(pd->isStartVertex(k)) start = x;
    if(pd->isGoalVertex(k)) goal = x;
  }

//#############################################################################
// Extract edges and weight
//#############################################################################
  typedef ListGraph::EdgeMap<double> CostMap;
  CostMap length(lg);

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

//#############################################################################
// Dijkstra on graph
//#############################################################################

  auto dstart = Dijkstra<ListGraph, CostMap>(lg, length);
  dstart.run(start);

  auto dgoal = Dijkstra<ListGraph, CostMap>(lg, length);
  dgoal.run(goal);

  all_paths.resize(N);
  all_paths_config.resize(N);
  all_vertices.resize(N);
  all_vertices_config.resize(N);

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
          const ob::State *s1 = pd->getVertex(v1i).getState();
          const ob::State *s2 = pd->getVertex(v2i).getState();
          Config q1 = cspace->OMPLStateToConfig(s1);
          Config q2 = cspace->OMPLStateToConfig(s2);
          L += (q1-q2).norm();

          shortest_path_idxs.push_back(v1i);
          shortest_path_idxs.push_back(v2i);
        }
      }
      all_paths.at(node_idx) = shortest_path_idxs;
      distance_shortest_path.at(node_idx) = L;

    }else{
      distance_shortest_path.at(node_idx) = dInf;
    }

    const ob::State *sn = pd->getVertex(node_idx).getState();
    all_vertices.at(node_idx) = node_idx;
    all_vertices_config.at(node_idx) = cspace->OMPLStateToConfig(sn);

  }
  if(distance_shortest_path.size()>0){
    uint idx_min = std::distance(std::begin(distance_shortest_path), std::min_element(std::begin(distance_shortest_path), std::end(distance_shortest_path)));
    uint idx_max = std::distance(std::begin(distance_shortest_path), std::max_element(std::begin(distance_shortest_path), std::end(distance_shortest_path)));
    min_distance_shortest_path = distance_shortest_path[idx_min];
    max_distance_shortest_path = distance_shortest_path[idx_max];
    std::cout << "min shortest path: " << min_distance_shortest_path << std::endl;
    std::cout << "max shortest path: " << max_distance_shortest_path << std::endl;
  }

  for(uint k = 0; k < all_paths.size(); k++){
    all_paths_config.at(k) = VertexPathToConfigPath(all_paths.at(k));
  }
}

//output:
//   cover_shortest_paths_vertex
//   cover_shortest_paths_config
//

void OnetopicPathSpaceModifier::ShortestPathsNonOnetopic(){
  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  typedef boost::graph_traits < PlannerDataGraph >::adjacency_iterator adjacency_iterator;

  const ob::SpaceInformationPtr si = pd->getSpaceInformation();
  Graph g = pd->toBoostGraph();

  VisibilityChecker path_checker(si, cspace);

  cover_shortest_paths_vertex.clear();
  cover_shortest_paths_config.clear();
  for(uint i = 0; i < all_vertices.size(); i++){
   
    assert(all_vertices.at(i) == i);
    Vertex v_current = all_vertices.at(i);
    double lk = distance_shortest_path.at(i);
    if(lk>=dInf) continue;

    const ob::State *state = pd->getVertex(v_current).getState();
    Config q = cspace->OMPLStateToConfig(state);

    std::pair<adjacency_iterator, adjacency_iterator> neighbors =
      boost::adjacent_vertices(vertex(v_current,g), g);

    bool neighborIsBetter = false;
    for(; neighbors.first != neighbors.second; ++neighbors.first){
      Vertex v_neighbor = *neighbors.first;
      double ln = distance_shortest_path.at(v_neighbor);

      if( ln < lk){
        neighborIsBetter = true;
        //extract shortest paths and compare their visibility:
        const std::vector<Vertex> pcur_idxs = all_paths.at(v_current);
        const std::vector<Vertex> pneighbor_idxs = all_paths.at(v_neighbor);
        neighborIsBetter = path_checker.isVisiblePathPath(*pd, pcur_idxs, pneighbor_idxs);
        if(neighborIsBetter) break; //don't check the other neighbors, we found at least one
      }
    }
    if(!neighborIsBetter){
      if(all_paths.at(i).size()>0){
        cover_shortest_paths_vertex.push_back(all_paths.at(i));
        cover_shortest_paths_config.push_back( VertexPathToConfigPath( all_paths.at(i) ) );
      }
    }
  }

  uint N = cover_shortest_paths_config.size();
  std::cout << "output paths: " << N << std::endl;

  //cover_vertices_config.resize(N);
  //cover_vertices.resize(N);

  //cover_edges_config.resize(N);
  //cover_edges.resize(N);

  //cover_paths.resize(N);
  //cover_paths_config.resize(N);

}
void OnetopicPathSpaceModifier::ComputeOnetopicCoverRoadmap(){

  const ob::SpaceInformationPtr si = pd->getSpaceInformation();
  VisibilityChecker path_checker(si, cspace);
  uint N = cover_shortest_paths_config.size();

  for(uint k = 0; k < N ; k++){

    std::vector<Vertex> single_cover_vertex;
    std::vector<Config> single_cover_vertex_config;

    std::vector<std::pair<Vertex,Vertex>> single_cover_edges;
    std::vector<std::pair<Config,Config>> single_cover_edges_config;

    std::vector<std::vector<Vertex>> single_cover_path;
    std::vector<std::vector<Config>> single_cover_path_config;


    for(uint j = 0; j < all_paths.size(); j++){
      bool isVisible = path_checker.isVisiblePathPath(*pd, all_paths.at(j), cover_shortest_paths_vertex.at(k));
      if(isVisible){
        single_cover_vertex.push_back(j);
        single_cover_path.push_back(all_paths.at(j));
        single_cover_vertex_config.push_back(VertexToConfig(j));
        single_cover_path_config.push_back( VertexPathToConfigPath(all_paths.at(j)) );
      }
    }

    std::cout << "onetopic cover " << k << " has " << single_cover_path.size() << " paths" << std::endl;

    cover_paths.push_back(single_cover_path);
    cover_paths_config.push_back(single_cover_path_config);
    cover_vertices.push_back(single_cover_vertex);
    cover_vertices_config.push_back(single_cover_vertex_config);
    cover_edges.push_back(single_cover_edges);
    cover_edges_config.push_back(single_cover_edges_config);

  }

  //for(uint k = 0; k < vertex_paths.size(); k++){
  //  std::vector<Vertex> single_cover_vertex;
  //  for(uint v_current = 0; v_current < Vidx.size(); v_current++){
  //    bool isVisible = path_checker.isVisiblePathVertex(pd_in, vertex_paths.at(k), v_current);
  //    if(isVisible){
  //      single_cover_vertex.push_back(v_current);
  //    }
  //  }
  //  cover_vertices.push_back(single_cover_vertex);

  //  std::vector<std::pair<Vertex,Vertex>> single_cover_edge;
  //  for(uint i = 0; i < single_cover_vertex.size(); i++){
  //    for(uint j = 0; j < single_cover_vertex.size(); j++){
  //      Vertex v_current = single_cover_vertex.at(i);
  //      Vertex v_next = single_cover_vertex.at(j);

  //      bool isVisible = path_checker.isVisibleVertexVertex(pd_in, v_current, v_next);
  //      if(isVisible){
  //        std::pair<Vertex,Vertex> e; 
  //        e.first = v_current;
  //        e.second = v_next;
  //        single_cover_edge.push_back(e);
  //      }
  //    }
  //  }
  //  cover_edges.push_back(single_cover_edge);
  //}
  //for(uint v_current = 0; v_current<  Vidx.size(); v_current++){

  //  std::pair<adjacency_iterator, adjacency_iterator> neighbors =
  //    boost::adjacent_vertices(vertex(v_current,g), g);

  //  for(; neighbors.first != neighbors.second; ++neighbors.first){
  //    Vertex v_next = *neighbors.first;
  //  //for(uint v_next = 0; v_next<  Vidx.size(); v_next++){
  //    //Vertex v_neighbor = *neighbors.first;
  //    bool isVisible = path_checker.isVisibleVertexVertex(pd_in, v_current, v_next);

  //    if(isVisible){
  //      Config q1 = config_vertices.at(v_current);
  //      Config q2 = config_vertices.at(v_next);
  //      //const ob::State *state = pd->getVertex(v_next).getState();
  //      //Config q2 = cspace->OMPLStateToConfig(state);
  //      std::pair<Config,Config> e; 
  //      e.first = q1;
  //      e.second = q2;
  //      config_edges.push_back(e);
  //    }
  //  }
  //}
}


Config OnetopicPathSpaceModifier::VertexToConfig( const Vertex& v){
  const ob::State *state = pd->getVertex(v).getState();
  Config q = cspace->OMPLStateToConfig(state);
  return q;
}
std::vector<Config> OnetopicPathSpaceModifier::VertexPathToConfigPath( const std::vector<Vertex> &path){
  const ob::SpaceInformationPtr si = pd->getSpaceInformation();

  std::vector<const ob::State*> states;
  for(uint k = 0; k < path.size(); k++){
    Vertex v = path.at(k);
    const ob::State *sk = pd->getVertex(v).getState();
    states.push_back(sk);
  }

  og::PathGeometric omplpath(si);
  for(uint k = 0; k < states.size(); k++){
    omplpath.append(states.at(k));
  }

  //og::PathSimplifier shortcutter(si);
  //shortcutter.shortcutPath(omplpath);

  //omplpath.interpolate();

  std::vector<ob::State *> interpolated_states = omplpath.getStates();
  std::vector<Config> keyframes;
  for(uint k = 0; k < interpolated_states.size(); k++)
  {
    ob::State *state = interpolated_states.at(k);
    Config q = cspace->OMPLStateToConfig(state);
    keyframes.push_back(q);
  }
  return keyframes;
}


//
//void OnetopicPathSpaceModifier::ComputeCoverVertices( ob::PlannerData& pd ){
//
//  cover_config_vertices.clear();
//  cover_config_edges.clear();
//
//  for(uint k = 0; k < cover_vertices.size(); k++){
//    std::vector< Config> single_cover_config;
//    for(uint j = 0; j < cover_vertices.at(k).size(); j++){
//      Vertex v = cover_vertices.at(k).at(j);
//      const ob::State *state = pd.getVertex(v).getState();
//      Config q = cspace->OMPLStateToConfig(state);
//      single_cover_config.push_back(q);
//    }
//    cover_config_vertices.push_back(single_cover_config);
//  }
//  for(uint k = 0; k < cover_edges.size(); k++){
//    std::vector< std::pair<Config,Config> > single_cover_edges;
//    for(uint j = 0; j < cover_edges.at(k).size(); j++){
//      Vertex v1 = cover_edges.at(k).at(j).first;
//      Vertex v2 = cover_edges.at(k).at(j).second;
//      const ob::State *s1 = pd.getVertex(v1).getState();
//      Config q1 = cspace->OMPLStateToConfig(s1);
//      const ob::State *s2 = pd.getVertex(v2).getState();
//      Config q2 = cspace->OMPLStateToConfig(s2);
//
//      std::pair<Config,Config> e;
//      e.first = q1;
//      e.second = q2;
//      single_cover_edges.push_back(e);
//    }
//    cover_config_edges.push_back(single_cover_edges);
//  }
//}

//std::vector< std::vector< const ob::State* >> OnetopicPathSpaceModifier::GetOMPLStatePaths(){
//  return cover_shortest_paths_omplstate;
//}
std::vector< std::vector< Config >> OnetopicPathSpaceModifier::GetShortestPathForEachCover(){
  return cover_shortest_paths_config;
}
std::vector< Config > OnetopicPathSpaceModifier::GetAllVertices(){
  return all_vertices_config;
}
std::vector< std::pair<Config,Config> > OnetopicPathSpaceModifier::GetAllEdges(){
  return all_edges_config;
}
std::vector< std::vector<Config>> OnetopicPathSpaceModifier::GetAllPaths(){
  return all_paths_config;
}
std::vector< std::vector< Config >> OnetopicPathSpaceModifier::GetCoverVertices(){
  return cover_vertices_config;
}
std::vector< std::vector< std::pair<Config,Config> >> OnetopicPathSpaceModifier::GetCoverEdges(){
  return cover_edges_config;
}
std::vector< std::vector<std::vector<Config> >> OnetopicPathSpaceModifier::GetCoverPaths(){
  return cover_paths_config;
}
