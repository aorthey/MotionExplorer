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

        void Clear();
        std::vector<simplex_t> GetVertices() const;
        void AddCoFace(Simplex* coface);

        std::vector<Simplex*> facets;
        std::vector<Simplex*> cofaces;
        std::vector<simplex_t> vertices;
      };

    }
  }
}
