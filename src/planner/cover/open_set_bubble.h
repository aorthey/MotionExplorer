#pragma once
#include "open_set.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

namespace cover{
  class OpenSetBubble: public OpenSet{
    public:
      OpenSetBubble(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_);

    protected:
      virtual double Distance(const ob::State *s_lhs, const ob::State *s_rhs) override;
      enum OpenSetType{ UNKNOWN, RN, SE2, SE3, SE3RN};
      OpenSetType type;
  };
};
