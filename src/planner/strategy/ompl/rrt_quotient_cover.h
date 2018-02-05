#pragma once
#include "rrt_quotient.h"
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/PDF.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    //unidirectional
    class RRTQuotientCover : public Quotient
    {
    public:
        RRTQuotientCover(const base::SpaceInformationPtr &si, Quotient *previous_ = nullptr);
        ~RRTQuotientCover();

        virtual bool SampleGraph(ob::State *q_random_graph) override;
    };
  }
}

