#pragma once
#include "klampt.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

//follows implementation as drafted by ademovic_2016
namespace cover{
  class OpenSet{
    public:
      OpenSet(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_);

      bool Contains(ob::State *sPrime);
      ob::State* IntersectionTowards(ob::State *sPrime);
      void IntersectionTowards(const ob::State *sGoal, ob::State *sIntersected);
      ob::State* GetCenter() const;
      double GetRadius() const;

    protected:
      double Distance(const ob::State *s_lhs, const ob::State *s_rhs);

      enum OpenSetType{ UNKNOWN, RN, SE2, SE3, SE3RN};
      OpenSetType type;

      ob::SpaceInformationPtr si;
      ob::State *sCenter;
      double dist_robot_env;
  };
};
