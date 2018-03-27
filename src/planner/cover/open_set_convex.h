#pragma once
#include "klampt.h"
#include "gui/gui_state.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <iris/iris.h>
#include <Eigen/Core>
namespace ob = ompl::base;

namespace cover{
  class OpenSetConvex{
    public:
      OpenSetConvex()=default;
      OpenSetConvex(iris::IRISRegion region_);
      virtual ~OpenSetConvex(){};

      void DrawGL(GUIState&);

      friend std::ostream& operator<< (std::ostream& out, const OpenSetConvex& set);

      iris::IRISRegion region;
    protected:


  };
};
