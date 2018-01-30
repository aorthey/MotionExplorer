#pragma once
#include "quotient.h"
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
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
    class RRTQuotient : public Quotient
    {
    public:
        //RRTQuotient(const base::SpaceInformationPtr &si, PRMQuotient *previous_);
        RRTQuotient(const base::SpaceInformationPtr &si, Quotient *previous_ = nullptr);
        ~RRTQuotient();

        void getPlannerData(base::PlannerData &data) const override;
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        void clear() override;

        void setRange(double distance)
        {
            maxDistance_ = distance;
        }

        double getRange() const
        {
            return maxDistance_;
        }
        void Init();

        virtual bool SampleGraph(ob::State *q_random_graph) override;

        template <template <typename T> class NN>
        void setNearestNeighbors()
        {
          if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
          clear();
          tStart_ = std::make_shared<NN<Configuration *>>();
          tGoal_ = std::make_shared<NN<Configuration *>>();
          setup();
        }
        virtual void Grow(double t=0);
        bool HasSolution() override{
          return isSolved;
        }

        void setup() override;
        virtual void CheckForSolution(ob::PathPtr &solution) override;

        virtual uint GetNumberOfVertices() override;
        virtual uint GetNumberOfEdges() override;

    protected:
        class Configuration
        {
        public:
          Configuration() = default;
          Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
          {

          }
          ~Configuration() = default;
          double parent_edge_weight;
          const base::State *root{nullptr};
          base::State *state{nullptr};
          Configuration *parent{nullptr};
        };

        typedef std::shared_ptr<NearestNeighbors<Configuration *>> TreeData;
        ompl::PDF<Configuration*> GetConfigurationPDF();

        struct TreeGrowingInfo
        {
            base::State *xstate;
            Configuration *xconfiguration;
            bool start;
        };

        enum GrowState
        {
            TRAPPED, ADVANCED, REACHED
        };

        /** \brief Free the memory allocated by this planner */
        void freeMemory();

        double distanceFunction(const Configuration *a, const Configuration *b) const
        {
            return si_->distance(a->state, b->state);
        }

        GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Configuration *rmotion);
        bool ConnectedToGoal();
        base::StateSamplerPtr sampler_;
        TreeData tStart_;
        TreeData tGoal_;
        TreeGrowingInfo tgi;

        double maxDistance_{0.};
        RNG rng_;
        std::pair<base::State *, base::State *> connectionPoint_;
        ob::GoalSampleableRegion *goal;

        ob::PathPtr ConstructSolution(Configuration *q_start, Configuration *q_goal);
        bool startTree;
        bool isTreeConnected{false};
        bool isSolved{false};
        Configuration *startConfiguration;
        Configuration *goalConfiguration;


    };
  }
}

