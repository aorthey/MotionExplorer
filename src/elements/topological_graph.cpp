#include "elements/topological_graph.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Fixed_alpha_shape_3.h>
#include <CGAL/Fixed_alpha_shape_vertex_base_3.h>
#include <CGAL/Fixed_alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_3.h>
#include <list>

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>

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

  as.get_alpha_shape_facets(std::back_inserter(facets),
                Fixed_alpha_shape_3::REGULAR);

  as.get_alpha_shape_facets(std::back_inserter(facets),
                Fixed_alpha_shape_3::SINGULAR);

  as.get_alpha_shape_edges(std::back_inserter(edges),
               Fixed_alpha_shape_3::SINGULAR);

  as.get_alpha_shape_vertices(std::back_inserter(vertices),
               Fixed_alpha_shape_3::SINGULAR);

  std::cout << " The 0-shape has : " << std::endl;
  std::cout << cells.size() << " interior tetrahedra" << std::endl;
  std::cout << facets.size() << " boundary facets" << std::endl;
  std::cout << edges.size()  << " singular edges" << std::endl;

  std::cout << vertices.size()  << " singular vertices" << std::endl;

  cmplx.V.clear();
  for( Vertex_handle v: vertices){
    //Vb vv = *v;
    K::Point_3 p = v->point();
    //std::cout << p[0] << "," << p[1] << "," << p[2] << std::endl;
    Vector3 vv(p[0],p[1],p[2]);
    cmplx.V.push_back(vv);
  }
}
SimplicialComplex& TopologicalGraph::GetSimplicialComplex(){
  return cmplx;
}
