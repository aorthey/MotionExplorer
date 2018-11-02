#include <string>
#include <iostream>
#include <fstream>
#include <CGAL/OFF_to_nef_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Surface_mesh.h>

int main(int argc, char* argv[]) 
{
  typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
  typedef CGAL::Nef_polyhedron_3<Kernel> Nef_3;
  //typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;

  // Mesh m;
  // // auto a = m.add_vertex(Kernel::Point_3(0,0,0));
  // // auto b = m.add_vertex(Kernel::Point_3(0,0,0));
  // // auto c = m.add_vertex(Kernel::Point_3(0,0,0));
  // Mesh::Vertex_index a = m.add_vertex(Kernel::Point_3(0,0,0));
  // Mesh::Vertex_index b = m.add_vertex(Kernel::Point_3(0,0,0));
  // Mesh::Vertex_index c = m.add_vertex(Kernel::Point_3(0,0,0));
  // m.add_face(a,b,c);

  Nef_3 N;
  //std::string fname = "/home/aorthey/git/orthoklampt/data/terrains/Lshape2.off";
  std::string fname = "/home/aorthey/Downloads/CGAL-4.10/demo/Polyhedron/data/cross.off";
  std::ifstream off_file(fname.c_str());
  assert(off_file.good());

  //std::ifstream off_file("/home/aorthey/Downloads/CGAL-4.10/demo/Polyhedron/data/cross.off");
  //std::cout << off_file.rdbuf() << std::endl;

  std::size_t discarded = CGAL::OFF_to_nef_3 (off_file, N, true);
  std::cout << "Nef vertices: "
           << N.number_of_vertices() << std::endl;
  std::cout << "Nef edges: "
           << N.number_of_edges() << std::endl;
  std::cout << "Nef facets: "
           << N.number_of_facets() << std::endl;
  std::cout << "Nef volumes: "
           << N.number_of_volumes() << std::endl;
  std::cout << "number of discarded facets: "
           << discarded << std::endl;
  return 0;
}
