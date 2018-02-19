#pragma once
#include "planner/strategy/strategy.h"
#include "planner/strategy/strategy_geometric.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
namespace ot = ompl::tools;

class StrategyGeometricMultiLevel: public StrategyGeometric{
  public:
    virtual void plan( const StrategyInput &input, StrategyOutput &output);

    StrategyGeometricMultiLevel();

    void BenchmarkFileToPNG(const std::string&);

};

