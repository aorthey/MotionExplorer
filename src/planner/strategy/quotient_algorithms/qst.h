#pragma once
#include "planner/strategy/quotientchart/quotient_chart.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    class QST : public og::QuotientChart
    {
      typedef og::QuotientChart BaseT;
    public:

      QST(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QST(void);
      virtual void clear() override;
      virtual void setup() override;
      void getPlannerData(ob::PlannerData &data) const override;

      void freeMemory();

      virtual uint GetNumberOfVertices() const override;
      virtual uint GetNumberOfEdges() const override;
      virtual void Grow(double t=0) override;
      virtual void Init() override;
      virtual bool HasSolution() override;
      virtual void CheckForSolution(ob::PathPtr &solution) override;

      
    protected:

      class Configuration
      {
      public:
        Configuration() = delete;
        Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
        {}
        Configuration(const base::SpaceInformationPtr &si, const ob::State *state_) : state(si->cloneState(state_))
        {}
        ~Configuration(){};
        double GetRadius() const{
          return openNeighborhoodRadius;
        }
        uint totalSamples{0}; //how many samples have been drawn from the edge between state and parent
        uint successfulSamples{0}; //how many of those samples have been successfully incorporated into the graph

        double parentEdgeWeight{0.0};
        base::State *state{nullptr};
        Configuration *parent{nullptr};
        bool isSufficientFeasible{false};
        double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
      };

      std::shared_ptr<NearestNeighbors<Configuration *>> cspace_tree;

      void AddState(ob::State *state);
      void AddConfiguration(Configuration *q);

      virtual void Sample(Configuration *q_random);
      virtual bool SampleGraph(ob::State*) override;

      //virtual Configuration* Nearest(Configuration *q);
      //virtual Configuration* Connect(Configuration *q_from, Configuration *q_to);
      virtual double Distance(const Configuration *q_from, const Configuration *q_to);

      virtual bool ConnectedToGoal(Configuration* q);
      void ConstructSolution(Configuration *q_goal);

      ob::GoalSampleableRegion *goal;

      Configuration *lastExtendedConfiguration{nullptr};
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};

      RNG rng_;
      double maxDistance_{0.};

      PDF<Configuration*> pdf_necessary_vertices;
      PDF<Configuration*> pdf_all_vertices;

    public:
      Configuration *lastSampled{nullptr};

    };
  }
}

