#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

namespace cover{
  class OpenSetHypersphere: public OpenSet{
    public:
      OpenSetHypersphere(CSpaceOMPL *cspace_, ob::State *s, double dist_robot_env_);
      virtual bool IsInside(ob::State *sPrime) override;
      void DrawGL(GUIState&) override;

      double GetRadius();
    protected:
      double Distance(const ob::State *s_lhs, const ob::State *s_rhs);
      double radius{0};
  };
};
