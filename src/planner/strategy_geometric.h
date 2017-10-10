#pragma once
#include "strategy.h"

//#include <boost/program_options.hpp>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace oa = ompl::app;
namespace ot = ompl::tools;


class StrategyGeometric: public Strategy{
  public:
    virtual void plan( const StrategyInput &input, CSpaceOMPL *cspace, StrategyOutput &output);

    StrategyGeometric();

    void DisableOnetopicReduction(){
      onetopic=false;
    }
  private:
    bool onetopic;

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
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
      const ob::SE3StateSpace::StateType *stateSE3 = state->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
      projection(0) = stateSE3->getX();
      projection(1) = stateSE3->getY();
      projection(2) = stateSE3->getZ();
    }
};
class WorkspaceProject0r : public ob::ProjectionEvaluator
{
  public:
    WorkspaceProject0r(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
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
    virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
    {
      const ob::RealVectorStateSpace::StateType *s = state->as<ob::RealVectorStateSpace::StateType>();
      projection(0) = s->values[0];
      projection(1) = s->values[1];
    }
};
