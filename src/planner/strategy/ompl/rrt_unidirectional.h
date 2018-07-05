#pragma once
#include "planner/strategy/quotient/quotient.h"
#include "planner/cspace/cover/open_set_hypersphere.h"
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/PDF.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

//based on <ompl/geometric/planners/rrt/RRT.h>
// unidirectional RRT

namespace ompl
{
  namespace geometric
  {

    class RRTUnidirectional : public og::Quotient
    {
    public:

      RRTUnidirectional(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~RRTUnidirectional(void);
      base::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
      void clear(void) override;
      void setup(void) override;
      void getPlannerData(ob::PlannerData &data) const override;

      double getRange() const;
      void setRange(double distance);
      void freeMemory();

      virtual uint GetNumberOfVertices() override;
      virtual uint GetNumberOfEdges() override;
      virtual void Grow(double t=0) override;
      virtual void Init() override;
      virtual bool HasSolution() override;
      virtual void CheckForSolution(ob::PathPtr &solution) override;

      
    protected:

      class Configuration
      {
      public:
        Configuration() = default;
        Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
        {}
        ~Configuration(){
          if(openset!=nullptr){
            delete openset;
          }
        }
        double GetRadius() const{
          if(openset!=nullptr){
            return openset->GetRadius();
          }else{
            return 0;
          }
        }
        uint totalSamples{0}; //how many samples have been drawn from the edge between state and parent
        uint successfulSamples{0}; //how many of those samples have been successfully incorporated into the graph

        double parent_edge_weight{0};
        base::State *state{nullptr};
        Configuration *parent{nullptr};
        cover::OpenSetHypersphere *openset{nullptr};
      };

      std::shared_ptr<NearestNeighbors<Configuration *>> G_;

      virtual void Sample(Configuration *q_random);
      virtual bool SampleGraph(ob::State*) override;
      PDF<RRTUnidirectional::Configuration*> GetConfigurationPDF();

      virtual Configuration* Nearest(Configuration *q_random);
      virtual Configuration* Connect(Configuration *q_near, Configuration *q_random);
      virtual double Distance(ob::State *s_lhs, ob::State *s_rhs);

      virtual bool ConnectedToGoal(Configuration* q);
      void ConstructSolution(Configuration *q_goal);

      ob::GoalSampleableRegion *goal;

      Configuration *lastExtendedConfiguration{nullptr};
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};

      RNG rng_;
      double goalBias_{.05};
      bool hasSolution{false};
      double maxDistance_{0.};
      base::StateSamplerPtr sampler_;

      //thickening of graph
      double epsilon{0};

      //see shkolnik_2011 for details
      double deltaCoverPenetration_;
    public:
      Configuration *lastSampled{nullptr};

    };
  }
}

