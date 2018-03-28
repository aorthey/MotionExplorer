#pragma once
#include "klampt.h"
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

namespace cover{
  class OpenSet{
    public:
      OpenSet(const ob::State *s);
      virtual ~OpenSet();
      virtual bool IsInside(ob::State *sPrime) = 0;

      const ob::State* GetCenter() const;
    protected:
      const ob::State *sCenter;
  };
};
