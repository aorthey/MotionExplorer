#include "simplicial_complex_simplex.h"
#include <algorithm>
#include <iostream>
using namespace ompl::geometric::topology;

Simplex::Simplex(std::vector<simplex_t> vertices_):
  vertices(vertices_) 
{
};

void Simplex::AddCoface(Simplex* coface)
{
  cofaces.push_back(coface);
}
std::vector<simplex_t> Simplex::GetVertices() const
{
  return vertices;
}
