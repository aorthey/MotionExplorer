#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <iris/iris.h>
namespace ob = ompl::base;

class CSpaceOMPL;

namespace cover{
  class OpenSetConvex: public OpenSet{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(CSpaceOMPL *cspace_, const ob::State *s, iris::IRISRegion region_);
      virtual ~OpenSetConvex(){};
      bool IsInside(ob::State *sPrime);

      void DrawGL(GUIState&) override;

      friend std::ostream& operator<< (std::ostream& out, const OpenSetConvex& set);

    protected:
      iris::IRISRegion region;
  };
};
