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
        void SetRadius(double radius){
          openNeighborhoodRadius = radius;
        }
        void clear(const base::SpaceInformationPtr &si){
          si->freeState(state);
        }

        uint totalSamples{0}; //how many samples have been drawn from the edge between state and parent
        uint successfulSamples{0}; //how many of those samples have been successfully incorporated into the graph

        double parentEdgeWeight{0.0};

        base::State *state{nullptr};
        Configuration *parent{nullptr};
        Configuration *coset{nullptr}; //the underlying coset this configuration belongs to (on the quotient-space)

        bool isSufficientFeasible{false};
        double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
      };


      void SetSubGraph( QuotientChart *sibling, uint k ) override;
      void AddState(ob::State *state);
      bool AddConfiguration(Configuration *q);
      bool sampleUniformOnNeighborhoodBoundary(Configuration *sample, const Configuration *center);

      virtual bool Sample(Configuration *q_random);
      virtual Configuration* SampleTree(ob::State*);

      Configuration* Nearest(Configuration *q) const;
      void Connect(const Configuration *q_from, Configuration *q_to);

      double Distance(const Configuration *q_from, const Configuration *q_to);
      double DistanceQ1(const Configuration *q_from, const Configuration *q_to);
      double DistanceX1(const Configuration *q_from, const Configuration *q_to);
      double DistanceTree(const Configuration *q_from, const Configuration *q_to);
      double DistanceOpenNeighborhood(const Configuration *q_from, const Configuration *q_to);

      void ConstructSolution(Configuration *q_goal);

      RNG rng_;

      double goalBias{0.05};
      std::shared_ptr<NearestNeighbors<Configuration *>> cover_tree;
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};
      PDF<Configuration*> pdf_necessary_vertices;
      PDF<Configuration*> pdf_all_vertices;

      Configuration *minimal_bounding_sphere{nullptr};
      double threshold_clearance{1e-8};

    public:

      std::shared_ptr<NearestNeighbors<Configuration *>> GetTree() const;
      Configuration* GetStartConfiguration() const;
      Configuration* GetGoalConfiguration() const;
      const PDF<Configuration*>& GetPDFNecessaryVertices() const;
      const PDF<Configuration*>& GetPDFAllVertices() const;
      double GetGoalBias() const;


      //Configuration *lastSampled{nullptr};

    };
  }
}

