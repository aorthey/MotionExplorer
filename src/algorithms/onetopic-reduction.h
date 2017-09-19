#include <ompl/base/Cost.h>
#include <ompl/base/PlannerData.h>
//#include <ompl/base/PlannerDataGraph.h>
//#include <ompl/base/spaces/SE3StateSpace.h>
//#include <ompl/base/ScopedState.h>
//#include <ompl/base/StateSpace.h>
#include <list>
#include <utility>
#include <map>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
typedef ob::Cost Cost;

static ob::OptimizationObjectivePtr getThresholdPathLength(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}
class LinearSegmentValidityChecker : public ob::StateValidityChecker
{
  public:
    LinearSegmentValidityChecker(const ob::SpaceInformationPtr &si, const ob::SpaceInformationPtr &si_path_, TopologicalGraph* tg, const ob::PlannerData& pd, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2):
      ob::StateValidityChecker(si), si_path(si_path_)
    {
      std::vector<Vector3> s1;
      std::vector<Vector3> s2;

      for(uint k = 0; k < p1.size(); k++){
        Vector3 v = tg->vertexIndexToVector(pd, p1.at(k));
        s1.push_back(v);
      }
      for(uint k = 0; k < p2.size(); k++){
        Vector3 v = tg->vertexIndexToVector(pd, p2.at(k));
        s2.push_back(v);
      }
      /////DEBUG

      path1 = PathPiecewiseLinearEuclidean::from_keyframes(s1);
      path1->Normalize();
      path2 = PathPiecewiseLinearEuclidean::from_keyframes(s2);
      path2->Normalize();
    }

    virtual bool isValid(const ob::State* state) const{

      const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
      double t1 = RnSpace->values[0];
      double t2 = RnSpace->values[1];

      Vector3 vv = path1->EvalVec3(t1);
      Vector3 ww = path2->EvalVec3(t2);

      const ob::StateSpacePtr space_path = si_path->getStateSpace();

      ob::State *q1 = space_path->allocState();
      ob::State *q2 = space_path->allocState();

      for(uint k = 0; k < 3; k++){
        q1->as<ob::RealVectorStateSpace::StateType>()->values[k] = vv[k];
        q2->as<ob::RealVectorStateSpace::StateType>()->values[k] = ww[k];
      }
    
      bool isfeasible = si_path->checkMotion(q1,q2);

      space_path->freeState(q1);
      space_path->freeState(q2);
      return isfeasible;

    }

  private:

    ob::SpaceInformationPtr si_path;
    PathPiecewiseLinearEuclidean *path1;
    PathPiecewiseLinearEuclidean *path2;

};

bool testVisibilityRRT(const ob::PlannerData& pd, const ob::SpaceInformationPtr &si_path_space, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2)
{
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

  ss.setStateValidityChecker(std::make_shared<LinearSegmentValidityChecker>(si_local, si_path_space,this,pd,p1,p2));

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


Vector3 TopologicalGraph::vertexIndexToVector(const ob::PlannerData& pd, const Vertex &v){
  ob::PlannerDataVertex vd = pd.getVertex(v);
  const ob::State* si = vd.getState();
  double x,y,z;
  const ob::RealVectorStateSpace::StateType *qomplRnSpace = si->as<ob::RealVectorStateSpace::StateType>();
  x = qomplRnSpace->values[0];
  y = qomplRnSpace->values[1];
  z = qomplRnSpace->values[2];
  Vector3 v3(x,y,z);
  return v3;
}
std::vector<Vector3> TopologicalGraph::vertexIndicesToVector(const ob::PlannerData& pd, const std::vector<Vertex> &v){

  std::vector<Vector3> output;
  for(uint i = 0; i < v.size(); i++){
    Vector3 v3 = vertexIndexToVector(pd, v.at(i));
    output.push_back(v3);
  }
  return output;
}

std::vector< std::vector< Vector3 >> ComputeShortestPathsLemon(ob::PlannerData& pd_in, const ob::OptimizationObjective& opt){

  using namespace lemon;

  const ob::SpaceInformationPtr si = pd_in.getSpaceInformation();
  Graph g = pd_in.toBoostGraph();

  uint N = pd_in.numVertices();

//#############################################################################
// Nodes boost to lemon
//#############################################################################
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

  cmplx.V_shortest_path.resize(N);
  cmplx.V.resize(N);
  cmplx.Vidx.resize(N);
  cmplx.distance_shortest_path.resize(N);

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
          std::cout << v1i << "->";
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
      std::cout << std::endl;
      Vector3 node_v3 = vertexIndexToVector(pd_in, node_idx);

      cmplx.V_shortest_path.at(node_idx) = shortest_path_idxs;
      cmplx.V.at(node_idx) = node_v3;
      //cmplx.Vidx.at(node_idx) = lg.id(node);
      cmplx.distance_shortest_path.at(node_idx) = L;

      std::cout << "vertex " << lg.id(node) << " length " << L << std::endl;
    }else{
      std::cout << "Found a non-connected vertex in graph -> vertex " << lg.id(node) << std::endl;
      cmplx.V.at(node_idx) = Vector3(0,0,0);
      //cmplx.Vidx.push_back(lg.id(node));
      cmplx.distance_shortest_path.at(node_idx) = dInf;
      //exit(0);
    }


  }
  if(cmplx.distance_shortest_path.size()>0){
    uint idx_min = std::distance(std::begin(cmplx.distance_shortest_path), std::min_element(std::begin(cmplx.distance_shortest_path), std::end(cmplx.distance_shortest_path)));
    uint idx_max = std::distance(std::begin(cmplx.distance_shortest_path), std::max_element(std::begin(cmplx.distance_shortest_path), std::end(cmplx.distance_shortest_path)));
    cmplx.min_distance_shortest_path = cmplx.distance_shortest_path[idx_min];
    cmplx.max_distance_shortest_path = cmplx.distance_shortest_path[idx_max];
    std::cout << "min shortest path: " << cmplx.min_distance_shortest_path << std::endl;
    std::cout << "max shortest path: " << cmplx.max_distance_shortest_path << std::endl;
  }
//################################################################################
  //simplify simplicial complex by removing vertices which belong to long paths

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);
  typedef boost::graph_traits < PlannerDataGraph >::adjacency_iterator adjacency_iterator;

  std::vector< std::vector< Config > > output;

  //cmplx.V.clear();
  
  for(uint i = 0; i < cmplx.Vidx.size(); i++){
    //std::cout << i << " <-> " << N-cmplx.Vidx.at(i) << std::endl;
   
    double lk = cmplx.distance_shortest_path.at(i);
    //uint k = cmplx.Vidx.at(i);
    Vertex current = i;

    std::pair<adjacency_iterator, adjacency_iterator> neighbors =
      boost::adjacent_vertices(vertex(current,g), g);
   
    bool neighborIsBetter = false;
    for(; neighbors.first != neighbors.second; ++neighbors.first)
      {
        uint idx = *neighbors.first;
        double ln = cmplx.distance_shortest_path.at(idx);
        if( ln < (lk - 1e-3)){
          neighborIsBetter = true;
          //found better neighbor

          //extract shortest paths and compare their visibility:
          Vertex v_current = i;
          Vertex v_neighbor = idx;

          // test with RRT
          const ob::SpaceInformationPtr si = pd_in.getSpaceInformation();
          const std::vector<Vertex> pcur_idxs = cmplx.V_shortest_path.at(i);
          const std::vector<Vertex> pneighbor_idxs = cmplx.V_shortest_path.at(idx);

          neighborIsBetter = testVisibilityRRT( pd_in, si, pcur_idxs, pneighbor_idxs);

          if(neighborIsBetter) break; //don't check the other neighbors, we found at least one
        }
      }
    if(neighborIsBetter){
      cmplx.V.at(i) = Vector3(0,0,0);
    }else{
      std::vector<Vertex> Vidxs = cmplx.V_shortest_path.at(i);
      if(Vidxs.size()>0){
        for(uint k = 0; k < Vidxs.size()-1; k++){
          Vector3 v1 = vertexIndexToVector(pd_in, Vidxs.at(k));
          Vector3 v2 = vertexIndexToVector(pd_in, Vidxs.at(k+1));
          //###################################################################
          cmplx.E.push_back(std::make_pair(v1,v2));
          //###################################################################
        }
      }
    }
  }

}

