#pragma once
#include "planner/strategy/strategy.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
namespace ot = ompl::tools;

class StrategyGeometricMultiLevel: public Strategy{
  public:
    virtual void plan( const StrategyInput &input, StrategyOutput &output);

    StrategyGeometricMultiLevel();

};

