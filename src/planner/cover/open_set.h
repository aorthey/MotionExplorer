#pragma once
#include "klampt.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

namespace cover{
  class OpenSet{
    public:
      OpenSet(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_);

      virtual bool Contains(ob::State *sPrime);
      virtual void IntersectionTowards(const ob::State *sGoal, ob::State *sIntersected);
      virtual ob::State* GetCenter() const;
      virtual double GetRadius() const;

    protected:
      virtual double Distance(const ob::State *s_lhs, const ob::State *s_rhs) = 0;

      enum OpenSetType{ UNKNOWN, RN, SE2, SE3, SE3RN};
      OpenSetType type;

      ob::SpaceInformationPtr si;
      ob::State *sCenter;

      double dist_robot_env;
  };
};
