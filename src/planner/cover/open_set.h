#pragma once
#include "klampt.h"
#include "planner/cspace/cspace.h"
#include "gui/gui_state.h"
#include "gui/colors.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

namespace cover{
  //@brief OpenSet represents an open set on the configuration space. OpenSet
  //has an associated configuration, which is called the center. center needs to
  //be a member of openset. The shape of the openset is determined by its child
  //class.
  class OpenSet{
    public:
      OpenSet(CSpaceOMPL *cspace_, const ob::State *s);
      virtual ~OpenSet();
      virtual bool IsInside(ob::State *sPrime) = 0;
      virtual void DrawGL(GUIState&) = 0;

      const ob::State* GetCenter() const;

      GLDraw::GLColor cOpenSet{magenta};

      friend std::ostream& operator<< (std::ostream& out, const OpenSet& set);
      virtual std::ostream& Print(std::ostream& out) const;

    protected:
      CSpaceOMPL *cspace;
      const ob::State *sCenter;
  };
};
