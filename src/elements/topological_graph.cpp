#include "elements/topological_graph.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_3.h>

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>

#include <list>
#include <utility>

typedef Math::Vector Config;
using namespace Topology;
using namespace Math3D;

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

TopologicalGraph::TopologicalGraph(ob::PlannerData& pd){
  std::cout << "TopologicalGraph" << std::endl;
  std::list<Weighted_point> lwp;

  for(uint i = 0; i < pd.numVertices(); i++){
    ob::PlannerDataVertex v = pd.getVertex(i);
    const ob::State* s = v.getState();
    double x,y,z;
    const ob::SE3StateSpace::StateType *sSE3 = s->as<ob::SE3StateSpace::StateType>();
    x = sSE3->getX();
    y = sSE3->getY();
    z = sSE3->getZ();
    lwp.push_back(Weighted_point(Bare_point( x,y,z), 0.3));
  }

  //build one alpha_shape  with alpha=0
  Fixed_alpha_shape_3  as(lwp.begin(), lwp.end(), 0);
  //explore the 0-shape - It is dual to the boundary of the union.
  std::list<Cell_handle> cells;
  std::list<Facet>       facets;
  std::list<Edge>        edges;
  std::list<Vertex_handle> vertices;

  as.get_alpha_shape_cells(std::back_inserter(cells),
               Fixed_alpha_shape_3::INTERIOR);
  as.get_alpha_shape_cells(std::back_inserter(cells),
               Fixed_alpha_shape_3::REGULAR);

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
    for(int k = 0; k < 4; k++){
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


  //cell handle
  //http://doc.cgal.org/latest/TDS_3/classTriangulationDataStructure__3_1_1Cell.html
  //The concept TriangulationDataStructure_3::Cell stores four Vertex_handles to its four vertices and four Cell_handles to its four neighbors. The vertices are indexed 0, 1, 2, and 3 in consistent order. The neighbor indexed i lies opposite to vertex i. 
  // Vertex_handle  vertex (int i) const
  //  Returns the vertex i of c. More...
  // 
  //int   index (Vertex_handle v) const
  //  Returns the index of vertex v in c. More...
  // 
  //bool  has_vertex (Vertex_handle v) const
  //  Returns true if v is a vertex of c.
  // 
  //bool  has_vertex (Vertex_handle v, int &i) const
  //  Returns true if v is a vertex of c, and computes its index i in c.
  // 
  //Cell_handle   neighbor (int i) const
  //  Returns the neighbor i of c. More...
  // 
  //int   index (Cell_handle n) const
  //  Returns the index corresponding to neighboring cell n. More...
  // 
  //bool  has_neighbor (Cell_handle n) const
  //  Returns true if n is a neighbor of c.
  // 
  //bool  has_neighbor (Cell_handle n, int &i) const
  //  Returns true if n is a neighbor of c, and computes its index i in c. 

  //K::Plane_3 pp = f->plane()
}
SimplicialComplex& TopologicalGraph::GetSimplicialComplex(){
  return cmplx;
}
