#include "elements/topological_graph.h"


//shortest path functions:
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>

#include <ompl/base/Cost.h>

#include <list>
#include <utility>

typedef Math::Vector Config;
using namespace Topology;
using namespace Math3D;


#undef CGAL_COMPUTATION
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
#endif

#undef BETTI_NUMBERS
#ifdef BETTI_NUMBERS
#include <SBL/GT/Betti_numbers_2.hpp>
  typedef SBL::GT::T_Betti_numbers_2<Fixed_alpha_shape_3>                        Betti_numbers_2;
#endif


TopologicalGraph::TopologicalGraph(const ob::PlannerData& pd, const ob::OptimizationObjective& obj){
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
    //std::cout << p.x() << std::endl;
    //std::cout << p.y() << std::endl;
    //std::cout << p.z() << std::endl;
    //std::cout << p.dimension() << std::endl;
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
#endif
#ifdef BETTI_NUMBERS
  Betti_numbers_2 betti;
  Betti_numbers_2::result_type res = betti(as);
  //std::cout << "Number of input spheres: " << as.number_of_vertices() << std::endl;
  //std::cout << "Betti numbers: " << res.get<0>() << " " << res.get<1>() << " " << res.get<2>() << std::endl;

  cmplx.betti_numbers.push_back(res.get<0>());
  cmplx.betti_numbers.push_back(res.get<1>());
  cmplx.betti_numbers.push_back(res.get<2>());
#endif

  ComputeShortestPaths(pd, obj);

}
SimplicialComplex& TopologicalGraph::GetSimplicialComplex(){
  return cmplx;
}

using Vertex = ob::PlannerData::Graph::Vertex;
using VIterator = ob::PlannerData::Graph::VIterator;
using EIterator = ob::PlannerData::Graph::EIterator;
using Graph = ob::PlannerData::Graph;

using namespace boost;
typedef boost::property_map<Graph, vertex_index_t>::type IndexMap;
typedef boost::iterator_property_map<Vertex*, IndexMap, Vertex, Vertex&> PredecessorMap;
typedef boost::iterator_property_map<int*, IndexMap, int, int&> DistanceMap;
typedef ob::Cost Cost;

void TopologicalGraph::ComputeShortestPaths(const ob::PlannerData& pd, const ob::OptimizationObjective& opt){

  Graph g = pd.toBoostGraph();

  EIterator ei, ei_end;
  VIterator vi, vi_end;
  ob::PlannerDataVertex vs = pd.getStartVertex(0);
  ob::PlannerDataVertex vg = pd.getGoalVertex(0);

  std::cout << "SimplicialComplex: vertices: " << num_vertices(g) << std::endl;
  std::cout << "                      edges: " << num_edges(g) << std::endl;

  for (tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi){
    //uint idx = get(index,*vi);
  }
  for(tie(ei, ei_end) = boost::edges(g); ei != ei_end;++ei){
    //std::cout << "(" << index[source(*ei,g)] << "->" << index[target(*ei,g)] << ")" << std::endl;
  }
  //Vertex start = *(vertices(g).first);
  //Vertex goal = *(vertices(g).second);

  Vertex start= pd.getStartIndex(0);
  Vertex goal = pd.getGoalIndex(0);

  std::vector<ob::Cost> dist_to_start(num_vertices(g));
  std::vector<ob::Cost> dist_to_goal(num_vertices(g));
  std::vector<ompl::base::PlannerData::Graph::Vertex> predecessors_to_start(num_vertices(g));
  std::vector<ompl::base::PlannerData::Graph::Vertex> predecessors_to_goal(num_vertices(g));
//################################################################################
  boost::dijkstra_shortest_paths(g, start, 
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
    .distance_zero(opt.identityCost()));
//################################################################################
  boost::dijkstra_shortest_paths(g, goal, 
    boost::predecessor_map(&predecessors_to_goal[0]).distance_map(&dist_to_goal[0])
    .distance_compare([&opt](Cost c1, Cost c2)
    {
     return opt.isCostBetterThan(c1, c2);
    })
    .distance_combine([](Cost, Cost c)
    {
     return c;
    })
    .distance_inf(opt.infiniteCost())
    .distance_zero(opt.identityCost()));
//################################################################################

  std::vector<Vertex> path;
  Vertex current=goal;

  while(current!=start) {
    path.push_back(current);
    std::cout << current << ":" << dist_to_start[current] << std::endl;  
    current=predecessors_to_start[current];
  }
  path.push_back(start);

  //Vertex current=start;

  //while(current!=goal) {
  //  path.push_back(current);
  //  std::cout << current << ":" << dist_to_goal[current] << std::endl;  
  //  current=predecessors_to_goal[current];
  //}
  //path.push_back(goal);

  cmplx.path.clear();
  std::vector<Vertex>::iterator it;
  for (it=path.begin(); it != path.end(); ++it) 
  {
    ob::PlannerDataVertex vi = pd.getVertex(*it);
    const ob::State* si = vi.getState();
    double x,y,z;
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = si->as<ob::RealVectorStateSpace::StateType>();
    x = qomplRnSpace->values[0];
    y = qomplRnSpace->values[1];
    z = qomplRnSpace->values[2];
    std::cout << *it << " : (" << x << "," << y << "," << z << ")" << std::endl;
    Vector3 v3(x,y,z);
    cmplx.path.push_back(v3);
    
  }
  std::cout << std::endl;

//################################################################################

  cmplx.V.clear();
  cmplx.E.clear();
  cmplx.F.clear();
  cmplx.T.clear();
  cmplx.distance_shortest_path.clear();
  cmplx.max_distance_shortest_path = 10000;

  std::cout << "distances and parents:" << std::endl;
  //graph_traits < graph_t >::vertex_iterator vi, vend;
  for(uint k = 0; k < predecessors_to_start.size(); k++){
    //std::cout << "distance(" << *vi << ") = " << dist_to_start[*vi] << ", ";

    ob::PlannerDataVertex vi = pd.getVertex(k);

    Vertex current=k;
    double dk = 0.0;
    while(current!=start) {
      dk += dist_to_start[current].value();
      current=predecessors_to_start[current];
    }
    current=k;
    //while(current!=goal) {
    //  dk += dist_to_goal[current].value();
    //  if(predecessors_to_goal[current]==current){
    //    dk=dInf;
    //    break;
    //  }
    //  current=predecessors_to_goal[current];
    //}

    if(dk<=cmplx.max_distance_shortest_path){
      std::cout << dk << ", ";
      const ob::State* si = vi.getState();
      double x,y,z;
      const ob::RealVectorStateSpace::StateType *qomplRnSpace = si->as<ob::RealVectorStateSpace::StateType>();
      x = qomplRnSpace->values[0];
      y = qomplRnSpace->values[1];
      z = qomplRnSpace->values[2];
      Vector3 v3(x,y,z);
      cmplx.V.push_back(v3);
      cmplx.distance_shortest_path.push_back(dk);
      //if(dk > cmplx.max_distance_shortest_path){
      //  cmplx.max_distance_shortest_path = dk;
      //}
    }
  }
  std::cout << "Found " << cmplx.V.size() << " eligible vertices." << std::endl;
  std::cout << "Start Vertices: " << pd.numStartVertices() << std::endl;
  std::cout << "Goal Vertices: " << pd.numGoalVertices() << std::endl;
  std::cout << predecessors_to_start.size()  << std::endl;
  std::cout << predecessors_to_goal.size()  << std::endl;
}
