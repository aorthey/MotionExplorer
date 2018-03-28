#pragma once
#include "klampt.h"
#include "open_set.h"
#include "gui/gui_state.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <iris/iris.h>
#include <Eigen/Core>
namespace ob = ompl::base;

namespace cover{
  class OpenSetConvex: public OpenSet{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(const ob::State *s, iris::IRISRegion region_);
      virtual ~OpenSetConvex(){};
      bool IsInside(ob::State *sPrime);

      void DrawGL(GUIState&);

      friend std::ostream& operator<< (std::ostream& out, const OpenSetConvex& set);

    protected:
      iris::IRISRegion region;

  };
};
