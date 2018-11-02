#pragma once
#include "cover.h"

namespace cover{

  //@brief: a cover of the cspace is a set of open sets. There are two marked
  //open sets, the ones containing the start and the goal configuration. Open
  //sets are allowed to overlap, but not to be nested inside other open sets.

  class CoverConvexPartition: public Cover{
    public:
      CoverConvexPartition();
      virtual void AddOpenSet( OpenSet* set );
  };
};
