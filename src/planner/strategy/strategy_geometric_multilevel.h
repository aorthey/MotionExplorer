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

static ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}
class StrategyGeometricMultiLevel: public StrategyGeometric{
  public:
    virtual void plan( const StrategyInput &input, StrategyOutput &output);

    StrategyGeometricMultiLevel();

    void BenchmarkFileToPNG(const std::string&);

};

