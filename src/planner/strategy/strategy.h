#pragma once
#include "planner/strategy/strategy_input.h"
#include "planner/strategy/strategy_output.h"
#include "planner/cspace/cspace.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

class Strategy{
  public:
    virtual void Plan( StrategyOutput &output) = 0;
    virtual void Step( StrategyOutput &output) = 0;
    virtual void Init( const StrategyInput &input) = 0;
    virtual void Clear() = 0;

    void BenchmarkFileToPNG(const std::string&);

    bool IsInitialized();

  protected:
    Strategy() = default;
    void setStateSampler(std::string sampler, ob::SpaceInformationPtr si);

    bool isInitialized{false};

    ob::PlannerPtr planner;

    double max_planning_time;
};

class SE3Project0r : public ob::ProjectionEvaluator
{
  public:
    SE3Project0r(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
    {
    }
    virtual unsigned int getDimension(void) const
    {
      return 3;
    }
    virtual void defaultCellSizes(void)
    {
      cellSizes_.resize(3);
      cellSizes_[0] = 0.1;
      cellSizes_[1] = 0.1;
      cellSizes_[2] = 0.1;
    }
    virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
        //ob::EuclideanProjection &projection) const
    {
      const ob::SE3StateSpace::StateType *stateSE3 = state->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
      projection(0) = stateSE3->getX();
      projection(1) = stateSE3->getY();
      projection(2) = stateSE3->getZ();
    }
};
class SE2Project0r : public ob::ProjectionEvaluator
{
  public:
    SE2Project0r(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
    {
    }
    virtual unsigned int getDimension(void) const
    {
      return 2;
    }
    virtual void defaultCellSizes(void)
    {
      cellSizes_.resize(2);
      cellSizes_[0] = 0.1;
      cellSizes_[1] = 0.1;
    }
    virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
        //ob::EuclideanProjection &projection) const
    {
      const ob::SE2StateSpace::StateType *stateSE2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
      projection(0) = stateSE2->getX();
      projection(1) = stateSE2->getY();
    }
};
