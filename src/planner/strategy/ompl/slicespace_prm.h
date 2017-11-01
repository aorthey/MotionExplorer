#pragma once

#include "slicespace.h"
#include "planner/cspace/cspace_factory.h"

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <utility>
#include <vector>
#include <map>

namespace ob = ompl::base;
namespace og = ompl::geometric;


class GoalRegionEdge: public ob::GoalRegion{
  public:
    GoalRegionEdge(const ob::SpaceInformationPtr &si): GoalRegion(si) {}

    virtual double distanceGoal(const ob::State *qompl) const override{
      const ob::RealVectorStateSpace::StateType *qomplRnSpace = qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
      //const ob::SO3StateSpace::StateType *qomplSO3 = qompl->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(1);
      double d = fabs(1.0 - qomplRnSpace->values[0]);
      return d;
    }
};

namespace ompl
{
  namespace base
  {
    OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class SliceSpacePRM : public base::Planner
    {
      public:
        SliceSpacePRM(RobotWorld* world_, const base::SpaceInformationPtr &si0, const ob::SpaceInformationPtr &si1);
        ~SliceSpacePRM() override;
        void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;
        void getPlannerData(base::PlannerData &data) const override;
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        SliceSpace* CreateNewSliceSpaceEdge(SliceSpace::Vertex v, SliceSpace::Vertex w, double yaw);
        void clear() override;
        void setup() override;

      protected:
        SliceSpace *S_0;
        SliceSpace *S_1;
        RobotWorld *world;

        ob::SpaceInformationPtr si_level0;
        ob::SpaceInformationPtr si_level1;
    };
  }
}

