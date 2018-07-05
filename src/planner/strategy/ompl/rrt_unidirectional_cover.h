#pragma once
#include "rrt_unidirectional.h"
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/PDF.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    class RRTUnidirectionalCover : public RRTUnidirectional
    {
    public:
      RRTUnidirectionalCover(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);

      ~RRTUnidirectionalCover(void);

      virtual void CheckForSolution(ob::PathPtr &solution) override;
      void getPlannerData(base::PlannerData &data) const override;

    protected:

      PDF<RRTUnidirectional::Configuration*> GetConfigurationPDF();
      virtual void Sample(RRTUnidirectional::Configuration*) override;
      virtual bool SampleGraph(ob::State*) override;
      virtual Configuration* Connect(Configuration *q_near, Configuration *q_random) override;
      //bool checkMotion(Configuration *q1, Configuration *q2);
      virtual bool ConnectedToGoal(Configuration* q = nullptr) override;

      bool IsInsideCover(Configuration*);

    };
  }
}

