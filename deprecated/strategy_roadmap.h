#pragma once
#include "planner/strategy/strategy.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


class StrategyRoadmap: public Strategy{
  typedef std::shared_ptr<og::PRM> PRMPtr;
  public:

    virtual void plan( const StrategyInput &input, StrategyOutput &output) override;

    StrategyRoadmap(CSpaceOMPL *cspace);

    int NumberConnectedComponents();
    bool hasFeasibleVertex();
    void planMore();

  private:

    PRMPtr roadmap_planner;

    int N_ConnectedComponents;
    int N_FeasibleVertices;

};

