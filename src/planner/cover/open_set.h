#pragma once
#include "klampt.h"
#include "planner/cspace/cspace.h"
#include "gui/gui_state.h"
#include "gui/colors.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

namespace cover{
  class OpenSet{
    public:
      OpenSet(CSpaceOMPL *cspace_, const ob::State *s);
      virtual ~OpenSet();
      virtual bool IsInside(ob::State *sPrime) = 0;
      virtual void DrawGL(GUIState&) = 0;

      const ob::State* GetCenter() const;

      GLDraw::GLColor cOpenSet{magenta};
    protected:
      CSpaceOMPL *cspace;
      const ob::State *sCenter;
  };
};
