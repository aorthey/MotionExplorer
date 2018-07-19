#include "simplicial_complex_simplex.h"
#include <algorithm>
#include <iostream>
using namespace ompl::geometric::topology;

Simplex::Simplex(std::vector<simplex_t> vertices_):
  vertices(vertices_) 
{
  //uint N = vertices.size();
  //if(N>2) comb(N,N-1);
};

// void Simplex::comb(int N, int K)
// {
//   std::string bitmask(K, 1); // K leading 1's
//   bitmask.resize(N, 0); // N-K trailing 0's
//   do {
//     std::vector<int> facet;
//     for (int i = 0; i < N; i++)
//     {
//       if (bitmask[i]) facet.push_back(vertices.at(i));
//     }
//     std::cout << "facet: " << facet << std::endl;
//     // Simplex *kfacet = simplicial_complex[facet];
//     // facets.push_back(kfacet);
//     //facets.push_back(facet);
//     //kfacet->AddCoFace(this);

//   } while (std::next_permutation(bitmask.begin(), bitmask.end()));
//   exit(0);
// }

void Simplex::Clear()
{
  for(uint k = 0; k < cofaces.size(); k++){
    cofaces.at(k)->Clear();
  }
  // for(uint k = 0; k < facets.size(); k++){
  //   Simplex *facet = facets.at(k);
  //   for(uint j = 0; j < facet->cofaces.size(); j++){
  //     Simplex *coface = facet->cofaces.at(j);
  //     if(coface == this)
  //     {
  //       facet->cofaces.erase(facet->cofaces.begin() + j);
  //       break;
  //     }
  //   }
  // }
  //vertices.clear();
  //no pointers should be left, we can remove this simplex
}

void Simplex::AddCoface(Simplex* coface)
{
  cofaces.push_back(coface);
}
std::vector<simplex_t> Simplex::GetVertices() const
{
  return vertices;
}
