#include "algorithms/onetopic_reduction.h"


const ob::State* vertexIndexToOMPLState(const ob::PlannerData& pd, const Vertex &v){
  ob::PlannerDataVertex vd = pd.getVertex(v);
  const ob::State* si = vd.getState();
  return si;
}
Vector3 OMPLStateToVector3(const ob::State* si){
  double x,y,z;
  const ob::RealVectorStateSpace::StateType *qomplRnSpace = si->as<ob::RealVectorStateSpace::StateType>();
  x = qomplRnSpace->values[0];
  y = qomplRnSpace->values[1];
  z = qomplRnSpace->values[2];
  Vector3 v3(x,y,z);
  return v3;
}
Config OMPLStateToConfig(const ob::State* si){
  double x,y,z;
  const ob::RealVectorStateSpace::StateType *qomplRnSpace = si->as<ob::RealVectorStateSpace::StateType>();
  x = qomplRnSpace->values[0];
  y = qomplRnSpace->values[1];
  z = qomplRnSpace->values[2];
  Config q;q.resize(3);
  q[0]=x; q[1]=y; q[2]=z;
  return q;
}
Vector3 vertexIndexToVector(const ob::PlannerData& pd, const Vertex &v){
  ob::PlannerDataVertex vd = pd.getVertex(v);
  const ob::State* si = vd.getState();
  return OMPLStateToVector3(si);
}
std::vector<Vector3> vertexIndicesToVector(const ob::PlannerData& pd, const std::vector<Vertex> &v){

  std::vector<Vector3> output;
  for(uint i = 0; i < v.size(); i++){
    Vector3 v3 = vertexIndexToVector(pd, v.at(i));
    output.push_back(v3);
  }
  return output;
}
#include <ompl/util/Console.h>

bool OnetopicPathSpaceModifier::testVisibilityRRT(const ob::PlannerData& pd, const ob::SpaceInformationPtr &si_path_space, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2)
{
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  //[0,1] x [0,1]
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(1);

  ob::StateSpacePtr space = (std::make_shared<ob::RealVectorStateSpace>(2));
  ob::RealVectorStateSpace *R2 = space->as<ob::RealVectorStateSpace>();

  R2->setBounds(bounds);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  start[0]=start[1]=0.0;
  goal[0]=goal[1]=1.0;

  og::SimpleSetup ss(space);
  const ob::SpaceInformationPtr si_local = ss.getSpaceInformation();

  ss.setStateValidityChecker(std::make_shared<LinearSegmentValidityChecker>(si_local, si_path_space,pd,p1,p2));

  ob::PlannerPtr ompl_planner = std::make_shared<og::RRT>(si_local);
  double epsilon_goalregion = 0.01;

  ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
  ss.setPlanner(ompl_planner);
  ss.setup();

  ////set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLength(si_local) );

  double max_planning_time=0.05;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  ss.solve(ptc);
  bool solved = ss.haveExactSolutionPath();
  return solved;
}

std::vector< std::vector< Vector3 >> OnetopicPathSpaceModifier::ComputeShortestPathsLemon(ob::PlannerData& pd_in, const ob::OptimizationObjective& opt){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Onetopic Reduction" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "input paths    : " << pd_in.numVertices() << std::endl;
  std::cout << "output paths   : " << std::flush;

//#############################################################################
// output
//#############################################################################
  std::vector< std::vector< Vector3 > > output;

  std::vector<Math3D::Vector3> V;
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
          //std::cout << v1i << "->";
          Vector3 v1 = vertexIndexToVector(pd_in, v1i);
          Vector3 v2 = vertexIndexToVector(pd_in, v2i);
          L += (v1-v2).norm();
          //###################################################################
          //cmplx.E.push_back(std::make_pair(v1,v2));
          //###################################################################
          shortest_path_idxs.push_back(v1i);
          shortest_path_idxs.push_back(v2i);
        }
      }
      //std::cout << std::endl;
      Vector3 node_v3 = vertexIndexToVector(pd_in, node_idx);

      V_shortest_path.at(node_idx) = shortest_path_idxs;
      V.at(node_idx) = node_v3;
      distance_shortest_path.at(node_idx) = L;

      //std::cout << "vertex " << lg.id(node) << " length " << L << std::endl;
    }else{
      //std::cout << "Found a non-connected vertex in graph -> vertex " << lg.id(node) << std::endl;
      V.at(node_idx) = Vector3(0,0,0);
      distance_shortest_path.at(node_idx) = dInf;
    }


  }
  if(distance_shortest_path.size()>0){
    uint idx_min = std::distance(std::begin(distance_shortest_path), std::min_element(std::begin(distance_shortest_path), std::end(distance_shortest_path)));
    uint idx_max = std::distance(std::begin(distance_shortest_path), std::max_element(std::begin(distance_shortest_path), std::end(distance_shortest_path)));
    min_distance_shortest_path = distance_shortest_path[idx_min];
    max_distance_shortest_path = distance_shortest_path[idx_max];
    //std::cout << "min shortest path: " << min_distance_shortest_path << std::endl;
    //std::cout << "max shortest path: " << max_distance_shortest_path << std::endl;
  }
//################################################################################
  //simplify simplicial complex by removing vertices which belong to long paths

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);
  typedef boost::graph_traits < PlannerDataGraph >::adjacency_iterator adjacency_iterator;
  
  for(uint i = 0; i < Vidx.size(); i++){
   
    double lk = distance_shortest_path.at(i);
    Vertex current = i;

    std::pair<adjacency_iterator, adjacency_iterator> neighbors =
      boost::adjacent_vertices(vertex(current,g), g);
   
    bool neighborIsBetter = false;
    for(; neighbors.first != neighbors.second; ++neighbors.first)
      {
        uint idx = *neighbors.first;
        double ln = distance_shortest_path.at(idx);
        if( ln < (lk - 1e-3)){
          neighborIsBetter = true;
          //found better neighbor

          //extract shortest paths and compare their visibility:
          Vertex v_current = i;
          Vertex v_neighbor = idx;

          // test with RRT
          const ob::SpaceInformationPtr si = pd_in.getSpaceInformation();
          const std::vector<Vertex> pcur_idxs = V_shortest_path.at(i);
          const std::vector<Vertex> pneighbor_idxs = V_shortest_path.at(idx);

          neighborIsBetter = testVisibilityRRT( pd_in, si, pcur_idxs, pneighbor_idxs);

          if(neighborIsBetter) break; //don't check the other neighbors, we found at least one
        }
      }
    if(neighborIsBetter){
      //V.at(i) = Vector3(0,0,0);
    }else{
      std::vector<Vertex> Vidxs = V_shortest_path.at(i);
      if(Vidxs.size()>0){
        std::vector<Vector3> path;
        std::vector<const ob::State*> omplstate_path;
        Vector3 vI = vertexIndexToVector(pd_in, Vidxs.at(0));
        const ob::State* s0 = vertexIndexToOMPLState(pd_in, Vidxs.at(0));

        path.push_back(vI);
        omplstate_path.push_back(s0);

        for(uint k = 0; k < Vidxs.size()-1; k++){
          Vector3 v1 = vertexIndexToVector(pd_in, Vidxs.at(k));
          Vector3 v2 = vertexIndexToVector(pd_in, Vidxs.at(k+1));
          const ob::State* sk = vertexIndexToOMPLState(pd_in, Vidxs.at(k+1));
          path.push_back(v2);
          omplstate_path.push_back(sk);

          //###################################################################
          //cmplx.E.push_back(std::make_pair(v1,v2));
          //###################################################################
        }
        output.push_back(path);
        omplstate_paths.push_back(omplstate_path);
      }
    }
  }
  std::cout << output.size() << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  return output;
}
OnetopicPathSpaceModifier::OnetopicPathSpaceModifier( ob::PlannerData& pd_in, const ob::OptimizationObjective& opt ){

  vector_paths = ComputeShortestPathsLemon(pd_in, opt);
  ComputeConfigPaths(pd_in);

}

void OnetopicPathSpaceModifier::ComputeConfigPaths( ob::PlannerData& pd ){
  const ob::SpaceInformationPtr si = pd.getSpaceInformation();


  for(uint i = 0; i < omplstate_paths.size(); i++){
    std::vector<const ob::State*> states = omplstate_paths.at(i);
    og::PathGeometric path(si);
    for(uint k = 0; k < states.size(); k++){
      path.append(states.at(k));
      //og::PathGeometric segment(si, states.at(k), states.at(k+1));
    }

    og::PathSimplifier shortcutter(si);
    shortcutter.shortcutPath(path);

    path.interpolate();

    std::vector<ob::State *> interpolated_states = path.getStates();
    std::vector<Config> keyframes;
    for(uint k = 0; k < interpolated_states.size(); k++)
    {
      ob::State *state = interpolated_states.at(k);
      Config q = OMPLStateToConfig(state);
      keyframes.push_back(q);
    }
    config_paths.push_back(keyframes);
  }
}

std::vector< std::vector< Vector3 >> OnetopicPathSpaceModifier::GetVectorPaths(){
  return vector_paths;
}
std::vector< std::vector< Config >> OnetopicPathSpaceModifier::GetConfigPaths(){
  return config_paths;
}
std::vector< std::vector< const ob::State* >> OnetopicPathSpaceModifier::GetOMPLStatePaths(){
  return omplstate_paths;
}
