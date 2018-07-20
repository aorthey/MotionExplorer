#pragma once
#include "common.h"

namespace ompl
{
  namespace geometric
  {
    namespace topology
    {
      typedef long unsigned int simplex_t;

      struct Simplex{
        Simplex(std::vector<simplex_t> vertices_);
        std::vector<simplex_t> GetVertices() const;
        void AddCoface(Simplex* coface);

        std::vector<std::vector<simplex_t>> edge_facets;
        std::vector<Simplex*> facets;
        std::vector<Simplex*> cofaces;
        std::vector<simplex_t> vertices;
      };

    }
  }
}
