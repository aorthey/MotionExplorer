#include "elements/topological_graph.h"
#include "elements/path_pwl_euclid.h"


//shortest path functions:
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/copy.hpp>

#include <ompl/base/Cost.h>
#include <list>
#include <utility>

typedef Math::Vector Config;
using namespace Topology;
using namespace Math3D;


//#define CGAL_COMPUTATION
#undef CGAL_COMPUTATION
//#undef BETTI_NUMBERS
#define BETTI_NUMBERS


#ifdef CGAL_COMPUTATION
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Regular_triangulation_vertex_base_3<K>        Vbb;
typedef CGAL::Fixed_alpha_shape_vertex_base_3<K,Vbb>        Vb;
typedef CGAL::Fixed_alpha_shape_cell_base_3<K>              Fb;
typedef CGAL::Triangulation_data_structure_3<Vb,Fb>         Tds;
typedef CGAL::Regular_triangulation_3<K,Tds>                Triangulation_3;
typedef CGAL::Fixed_alpha_shape_3<Triangulation_3>          Fixed_alpha_shape_3;

typedef Fixed_alpha_shape_3::Cell_handle                    Cell_handle;
typedef Fixed_alpha_shape_3::Vertex_handle                  Vertex_handle;
typedef Fixed_alpha_shape_3::Facet                          Facet;
typedef Fixed_alpha_shape_3::Edge                           Edge;
typedef K::Weighted_point_3                                 Weighted_point;
typedef K::Point_3                                          Bare_point;
#ifdef BETTI_NUMBERS
#include <SBL/GT/Betti_numbers_2.hpp>
  typedef SBL::GT::T_Betti_numbers_2<Fixed_alpha_shape_3>                        Betti_numbers_2;
#endif
#endif

using Vertex = ob::PlannerData::Graph::Vertex;
using VIterator = ob::PlannerData::Graph::VIterator;
using EIterator = ob::PlannerData::Graph::EIterator;
using Graph = ob::PlannerData::Graph;

using namespace boost;
typedef boost::property_map<Graph, vertex_index_t>::type IndexMap;
typedef boost::iterator_property_map<Vertex*, IndexMap, Vertex, Vertex&> PredecessorMap;
typedef boost::iterator_property_map<int*, IndexMap, int, int&> DistanceMap;
typedef ob::Cost Cost;

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

static ob::OptimizationObjectivePtr getThresholdPathLength(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}
namespace ompl{
  namespace base{
    typedef std::shared_ptr<StateValidityChecker> StateValidityCheckerPtr;
  }
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

      //s1.clear();s2.clear();
      //s1.push_back(Vector3(0,-2,0));
      //s1.push_back(Vector3(-2,-2,0));
      //s1.push_back(Vector3(0,+2,0));

      //s2.push_back(Vector3(0,-2,0));
      //s2.push_back(Vector3(-2,0,0));
      //s2.push_back(Vector3(0,+2,0));


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
      //std::cout << "[" << t1 << "] " << vv << " -> "  << "[" << t2 << "] " << ww << "(" << (isfeasible?"feasible":"invalid") << ")" << std::endl;

      space_path->freeState(q1);
      space_path->freeState(q2);
      return isfeasible;

    }

  private:

    ob::SpaceInformationPtr si_path;
    PathPiecewiseLinearEuclidean *path1;
    PathPiecewiseLinearEuclidean *path2;

};

bool TopologicalGraph::testVisibilityRRT(const ob::PlannerData& pd, const ob::SpaceInformationPtr &si_path_space, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2)
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

  double max_planning_time=0.03;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  ss.solve(ptc);
  bool solved = ss.haveExactSolutionPath();
  //if(!solved){
  //  std::cout << "NOT SOLVED" << std::endl;
  //}
  //exit(0);
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
std::vector<Vertex> TopologicalGraph::shortestPath(const Vertex start, const Vertex middle, const Vertex goal, 
  const std::vector<Vertex> &predecessors_to_start, const std::vector<Vertex> &predecessors_to_goal)
{
  Vertex current = middle;

  std::vector<Vertex> path_to_start;

  while(current!=start) {
    if(predecessors_to_start[current]==current){
      std::cout << "TopologicalGraph: no path from vertex " << middle << " to start vertex " << start << std::endl;
      exit(0);
    }
    current=predecessors_to_start[current];
    path_to_start.push_back(current);
  }

  std::vector<Vertex> path(path_to_start.rbegin(), path_to_start.rend());

  current=middle;
  path.push_back(current);

  while(current!=goal) {
    if(predecessors_to_goal[current]==current){
      std::cout << "TopologicalGraph: no path from vertex " << current << " to goal vertex " << goal << std::endl;
      exit(0);
    }
    current=predecessors_to_goal[current];
    path.push_back(current);
  }
  return path;

}
template<typename T>
std::vector<std::vector<T> > TopologicalGraph::extractFacetsBetweenPaths( const std::vector<T> &p1, const std::vector<T> &p2)
{
  if(p1.size()<2){
    std::cout << "no facets between paths" << std::endl;
    exit(0);
  }
  if(p1.size() < p2.size()){
    return extractFacetsBetweenPaths(p2,p1);
  }

  //assume p1.size => p2.size
  std::vector<std::vector<T> > facets;

  uint k = 0;

  // v1 ----- v2 ---- v3 ---- .... --- vI
  //
  // w1 ----- w2 ---- w3 ---- .... --- vK
  std::cout << std::string(80, '-') << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << p1.size() << "vs" << p2.size() << std::endl;
  for(uint i = 0; i < p1.size()-1; i++){
    T v1 = p1.at(i);
    T v2 = p1.at(i+1);

    T w1 = p2.at(k);
    std::vector<T> facetVVW;
    facetVVW.push_back(v1);
    facetVVW.push_back(v2);
    facetVVW.push_back(w1);
    facets.push_back(facetVVW);
    // v1-v2
    //  |/ 
    // w1  w2
    if(k < (p2.size()-1)){
      k++;
      T w2 = p2.at(k);
      std::vector<T> facetVWW;
      facetVWW.push_back(v2);
      facetVWW.push_back(w1);
      facetVWW.push_back(w2);
      facets.push_back(facetVWW);
      // v1-v2
      //  |/|
      // w1-w2
    }
  }
  return facets;
}

TopologicalGraph::TopologicalGraph(ob::PlannerData& pd, const ob::OptimizationObjective& obj){
#ifdef CGAL_COMPUTATION
  std::cout << "TopologicalGraph" << std::endl;
  std::list<Weighted_point> lwp;

  if(pd.numVertices()<=1) return;

  for(uint i = 0; i < pd.numVertices(); i++){
    ob::PlannerDataVertex v = pd.getVertex(i);
    const ob::State* s = v.getState();
    double x,y,z;
    //const ob::SE3StateSpace::StateType *sSE3 = s->as<ob::SE3StateSpace::StateType>();
    //x = sSE3->getX();
    //y = sSE3->getY();
    //z = sSE3->getZ();
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = s->as<ob::RealVectorStateSpace::StateType>();
    x=qomplRnSpace->values[0];
    y=qomplRnSpace->values[1];
    z=qomplRnSpace->values[2];
    lwp.push_back(Weighted_point(Bare_point( x,y,z), 0.1));
  }

  Fixed_alpha_shape_3  as(lwp.begin(), lwp.end(), 0);
  std::list<Cell_handle> cells;
  std::list<Facet>       facets;
  std::list<Edge>        edges;
  std::list<Vertex_handle> vertices;

  as.get_alpha_shape_cells(std::back_inserter(cells),
               Fixed_alpha_shape_3::INTERIOR);
  as.get_alpha_shape_cells(std::back_inserter(cells),
               Fixed_alpha_shape_3::REGULAR);
  as.get_alpha_shape_cells(std::back_inserter(cells),
               Fixed_alpha_shape_3::SINGULAR);

  as.get_alpha_shape_facets(std::back_inserter(facets),
                Fixed_alpha_shape_3::REGULAR);
  as.get_alpha_shape_facets(std::back_inserter(facets),
                Fixed_alpha_shape_3::SINGULAR);

  as.get_alpha_shape_edges(std::back_inserter(edges),
               Fixed_alpha_shape_3::REGULAR);
  as.get_alpha_shape_edges(std::back_inserter(edges),
               Fixed_alpha_shape_3::SINGULAR);

  as.get_alpha_shape_vertices(std::back_inserter(vertices),
               Fixed_alpha_shape_3::REGULAR);
  as.get_alpha_shape_vertices(std::back_inserter(vertices),
               Fixed_alpha_shape_3::SINGULAR);

  std::cout << " The 0-shape has : " << std::endl;
  std::cout << cells.size() << " interior tetrahedra" << std::endl;
  std::cout << facets.size() << " boundary facets" << std::endl;
  std::cout << edges.size()  << " singular edges" << std::endl;

  std::cout << vertices.size()  << " regular vertices" << std::endl;

  cmplx.V.clear();
  for( Vertex_handle v: vertices){
    //Documentation: http://doc.cgal.org/latest/Kernel_23/classCGAL_1_1Point__3.html
    K::Point_3 p = v->point();
    Vector3 vv(p[0],p[1],p[2]);
    cmplx.V.push_back(vv);
  }
  cmplx.E.clear();
  for( Edge e: edges){
    //K::Line_3 l = e->line();
    Cell_handle ce = e.first;
    uint idx_v1 = e.second;
    uint idx_v2 = e.third;

    K::Point_3 p1 = ce->vertex(idx_v1)->point();
    K::Point_3 p2 = ce->vertex(idx_v2)->point();
    Vector3 v1(p1[0],p1[1],p1[2]);
    Vector3 v2(p2[0],p2[1],p2[2]);

    cmplx.E.push_back(std::make_pair(v1,v2));
  }
  cmplx.F.clear();
  for( Facet f: facets){
    Cell_handle cf = f.first;
    uint idx_vf = f.second; //idx of vertex opposite of facet in cell
    std::vector<Vector3> V;
    for(uint k = 0; k < 4; k++){
      if(k==idx_vf) continue;
      K::Point_3 pk = cf->vertex(k)->point();
      Vector3 vk(pk[0],pk[1],pk[2]);
      V.push_back(vk);
    }
    cmplx.F.push_back(V);
  }
  cmplx.T.clear();
  for( Cell_handle c: cells){
    std::vector<Vector3> V;
    for(int k = 0; k < 4; k++){
      K::Point_3 pk = c->vertex(k)->point();
      Vector3 vk(pk[0],pk[1],pk[2]);
      V.push_back(vk);
    }
    cmplx.T.push_back(V);
  }
//http://sbl.inria.fr/doc/Betti_numbers-user-manual.html
#ifdef BETTI_NUMBERS
  Betti_numbers_2 betti;
  Betti_numbers_2::result_type res = betti(as);
  //std::cout << "Number of input spheres: " << as.number_of_vertices() << std::endl;
  //std::cout << "Betti numbers: " << res.get<0>() << " " << res.get<1>() << " " << res.get<2>() << std::endl;

  cmplx.betti_numbers.push_back(res.get<0>());
  cmplx.betti_numbers.push_back(res.get<1>());
  cmplx.betti_numbers.push_back(res.get<2>());
#endif
#endif

  ComputeShortestPaths(pd, obj);

}
SimplicialComplex& TopologicalGraph::GetSimplicialComplex(){
  return cmplx;
}

void TopologicalGraph::ComputeShortestPaths(ob::PlannerData& pd_in, const ob::OptimizationObjective& opt){

  Graph g = pd_in.toBoostGraph();
  PlannerDataGraph gb(g);
  PlannerDataGraphUndirected gu;

  boost::copy_graph(gb, gu);//NOT_OPTIMAL: avoid copying the whole graph

  EIterator ei, ei_end;
  VIterator vi, vi_end;
  ob::PlannerDataVertex vs = pd_in.getStartVertex(0);
  ob::PlannerDataVertex vg = pd_in.getGoalVertex(0);
  Vertex start= pd_in.getStartIndex(0);
  Vertex goal = pd_in.getGoalIndex(0);

  std::cout << "PlannerDataGraph:  vertices: " << num_vertices(g) << std::endl;
  std::cout << "                      edges: " << num_edges(g) << std::endl;
  std::cout << "PlannerDataGraphU: vertices: " << num_vertices(gu) << std::endl;
  std::cout << "                      edges: " << num_edges(gu) << std::endl;

  using UVertex = boost::graph_traits<PlannerDataGraphUndirected>::vertex_descriptor;
  using UEdge = boost::graph_traits<PlannerDataGraphUndirected>::edge_descriptor;

  for (tie(vi, vi_end) = boost::vertices(gu); vi != vi_end; ++vi){
    //uint idx = get(index,*vi);

    boost::graph_traits<PlannerDataGraphUndirected>::out_edge_iterator e, e_end;
    for(tie(e, e_end) = boost::out_edges(*vi, gu); e != e_end;++e){


      UVertex e1 = source(*e,gu);
      UVertex e2 = target(*e,gu);
      property_map<PlannerDataGraphUndirected, edge_weight_t>::type weight = get(edge_weight, gu);

      Vector3 v1 = vertexIndexToVector(pd_in, e1);
      Vector3 v2 = vertexIndexToVector(pd_in, e2);

      double d = (v1-v2).norm();

      std::cout << "(" << source(*e, gu) << "," << target(*e, gu) << ") -> " << get(weight,*e) << " = " << d<< std::endl;

      //cmplx.E.push_back(std::make_pair(v1,v2));
    }

  }

  std::vector<ob::Cost> dist_to_start(num_vertices(g));
  std::vector<ob::Cost> dist_to_goal(num_vertices(g));

  std::vector<UVertex> predecessors_to_start(num_vertices(gu));
  std::vector<UVertex> predecessors_to_goal(num_vertices(gu));

  UVertex ustart = start;
  UVertex ugoal = goal;

//################################################################################
  boost::dijkstra_shortest_paths(gu, start,
    boost::predecessor_map(&predecessors_to_start[0]).distance_map(&dist_to_start[0])
    .distance_compare([&opt](Cost c1, Cost c2)
    {
     return opt.isCostBetterThan(c1, c2);
    })
    .distance_combine([](Cost, Cost c)
    {
     return c;
    })
    .distance_inf(opt.infiniteCost())
    .distance_zero(opt.identityCost())
    );
//################################################################################

  std::cout << "predecessors_to_start " << predecessors_to_start.size()   << std::endl;
  for(uint k = 0; k < predecessors_to_start.size()  ; k++){
    Vector3 v3 = vertexIndexToVector(pd_in, k);
    cmplx.V.push_back(v3);
    cmplx.distance_shortest_path.push_back(dist_to_start[k].value());
    std::cout << "vertex " << k << " dist start " << dist_to_start[k] << std::endl;
  }

  for(uint k = 0; k < predecessors_to_start.size(); k++){

    if( predecessors_to_start[k] != k ){
      Vector3 v1 = vertexIndexToVector(pd_in, k);
      Vector3 v2 = vertexIndexToVector(pd_in, predecessors_to_start[k]);

      cmplx.E.push_back(std::make_pair(v1,v2));
    }

  }
  uint idx_min = std::distance(std::begin(cmplx.distance_shortest_path), std::min_element(std::begin(cmplx.distance_shortest_path), std::end(cmplx.distance_shortest_path)));
  uint idx_max = std::distance(std::begin(cmplx.distance_shortest_path), std::max_element(std::begin(cmplx.distance_shortest_path), std::end(cmplx.distance_shortest_path)));
  cmplx.min_distance_shortest_path = cmplx.distance_shortest_path[idx_min];
  cmplx.max_distance_shortest_path = cmplx.distance_shortest_path[idx_max];
  return;
//################################################################################
  //extract cumulative distance for each vertex (distance to start + distance to
  //goal)

  cmplx.V.clear();
  cmplx.E.clear();
  //cmplx.F.clear();
  //cmplx.T.clear();
  cmplx.distance_shortest_path.clear();
  cmplx.min_distance_shortest_path = dInf;
  cmplx.max_distance_shortest_path = 0;

  for(uint k = 0; k < pd_in.numVertices(); k++){

    ob::PlannerDataVertex vi = pd_in.getVertex(k);
    Vertex current = pd_in.vertexIndex(vi);

    double dk = 0.0;
    bool unreachable_vertex = false;
    while(current!=start) {
      dk += dist_to_start[current].value();
      if(predecessors_to_start[current]==current){
        std::cout << "iter" << k << ": TopologicalGraph: no path from vertex " << current << " to start vertex " << start << std::endl;
        exit(0);
        unreachable_vertex = true;
        break;
      }
      current=predecessors_to_start[current];
    }
    current=k;
    while(current!=goal) {
      dk += dist_to_goal[current].value();
      if(predecessors_to_goal[current]==current){
        std::cout << "TopologicalGraph: no path from vertex " << current << " to goal vertex " << goal << std::endl;
        exit(0);
        unreachable_vertex = true;
        break;
      }
      current=predecessors_to_goal[current];
    }
    if(unreachable_vertex) continue;
    Vector3 v3 = vertexIndexToVector(pd_in, k);
    cmplx.V.push_back(v3);
    cmplx.distance_shortest_path.push_back(dk);

    if(dk > cmplx.max_distance_shortest_path)
      cmplx.max_distance_shortest_path = dk;
    if(dk < cmplx.min_distance_shortest_path)
      cmplx.min_distance_shortest_path = dk;
  }
  std::cout << "Found " << cmplx.V.size() << " eligible vertices." << std::endl;
//################################################################################

  std::vector<Vertex> path_idxs = shortestPath(start, start, goal, predecessors_to_start, predecessors_to_goal);
  cmplx.path = vertexIndicesToVector(pd_in, path_idxs);

//################################################################################
  //simplify simplicial complex by removing vertices which belong to long paths

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);
  typedef boost::graph_traits < PlannerDataGraphUndirected >::adjacency_iterator adjacency_iterator;

  //cmplx.V.clear();
  for(uint i = 0; i < pd_in.numVertices(); i++){
   
    double lk = cmplx.distance_shortest_path.at(i);

    Vertex current=i;
    std::pair<adjacency_iterator, adjacency_iterator> neighbors =
      boost::adjacent_vertices(vertex(current,gu), gu);
   
    std::cout << current << "(" << lk << ") -> ";
    bool neighborIsBetter = false;
    for(; neighbors.first != neighbors.second; ++neighbors.first)
      {
        uint idx = *neighbors.first;//index[*neighbors.first];
        double ln = cmplx.distance_shortest_path.at(idx);
        std::cout << idx << "(" << ln << ") ";
        if( ln < (lk - 1e-3)){
          neighborIsBetter = true;
          //found better neighbor

          //extract shortest paths and compare their visibility:
          Vertex v_current = i;
          Vertex v_neighbor = idx;

          //extract shortest path 1: start -> v_current -> goal
          std::vector<Vertex> pcur_idxs = shortestPath(start, v_current, goal, predecessors_to_start, predecessors_to_goal);
          std::vector<Vector3> pcur = vertexIndicesToVector(pd_in, pcur_idxs);

          std::vector<Vertex> pneighbor_idxs = shortestPath(start, v_neighbor, goal, predecessors_to_start, predecessors_to_goal);
          std::vector<Vector3> pneighbor = vertexIndicesToVector(pd_in, pneighbor_idxs);

          // test with RRT
          const ob::SpaceInformationPtr si = pd_in.getSpaceInformation();
          neighborIsBetter = testVisibilityRRT( pd_in, si, pcur_idxs, pneighbor_idxs);

          //if(!neighborIsBetter){
          //  std::cout << std::string(80, '-') << std::endl;
          //  std::cout << std::string(80, '-') << std::endl;
          //  std::cout << "Neighborpath is better? " << (neighborIsBetter?"Yes":"No") << std::endl;
          //  std::cout << std::string(80, '-') << std::endl;
          //  std::cout << std::string(80, '-') << std::endl;

          //  for(uint j = 0; j < pcur.size()-1; j++){
          //    cmplx.E.push_back(std::make_pair(pcur.at(j),pcur.at(j+1)));
          //  }
          //  for(uint j = 0; j < pneighbor.size()-1; j++){
          //    cmplx.E.push_back(std::make_pair(pneighbor.at(j),pneighbor.at(j+1)));
          //  }

          //  return;
          //}else{
          //  std::cout << "(del) " << i;
          //}


          break;
        }
      }
    std::cout << std::endl;

    if(!neighborIsBetter){
      //found local shortest path -> add to complex
      //double ln = cmplx.distance_shortest_path.at(idx);
    //Vertex current=i;
    std::vector<Vertex> pcur_idxs = shortestPath(start, current, goal, predecessors_to_start, predecessors_to_goal);
    std::vector<Vector3> pcur = vertexIndicesToVector(pd_in, pcur_idxs);

    static double dstep = 0.5;

    static double dd = 0;
    for(uint k = 0; k < pcur.size()-1; k++){
      Vector3 v1 = pcur.at(k);
      Vector3 v2 = pcur.at(k+1);
      v1[2]=dd;
      v2[2]=dd;
      cmplx.E.push_back(std::make_pair(v1,v2));
    }

    cmplx.V.at(i)[2]=dd;
    cmplx.V.at(i)[0]+=0.05;

    dd+=dstep;



    }else{
      //cmplx.V.at(i)[2]=dstep;
      //= Vector3(0,0,0);
    }

  }
  std::cout << "Found " << cmplx.V.size() << " eligible vertices." << std::endl;
}
