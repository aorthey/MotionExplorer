#pragma once
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>

//Simplicialcomplex consists of
//  V vertices
//  E edges
//  F faces
//  T tetrahedras
//
// path shortest path between nodes
//

struct SimplicialComplex{
  std::vector<Math3D::Vector3> V;
  std::vector<std::pair<Math3D::Vector3,Math3D::Vector3>> E;
  std::vector<std::vector<Math3D::Vector3> > F;
  std::vector<std::vector<Math3D::Vector3> > T;
  std::vector<Math3D::Vector3> path;
  std::vector<double> distance_shortest_path; //length of the shortest path from start to goal >including< vertex i
  double max_distance_shortest_path;
  std::vector<int> betti_numbers;
};

