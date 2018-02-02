#pragma once
#include "cover_open_set.h"

namespace cover{
  class Cover{
    public:
      Cover();

      bool IsInsideCover(ob::State *s);
      void ComputeCoverAt(ob::State *x);

    protected:
      CSpace *cspace;
      std::vector<OpenSet> sets;
  };
};
