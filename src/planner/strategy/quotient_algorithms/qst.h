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
        double GetRadius() const
        {
          return openNeighborhoodRadius;
        }
        void SetRadius(double radius)
        {
          openNeighborhoodRadius = radius;
        }
        void clear(const base::SpaceInformationPtr &si)
        {
          si->freeState(state);
        }
        void SetPDFElement(void *element_)
        {
          element = element_;
        }
        void* GetPDFElement()
        {
          return element;
        }
        double GetImportance()
        {
          return openNeighborhoodRadius;
          //return ((double)number_successful_expansions+1)/((double)number_attempted_expansions+2);
          //return 1.0/((double)number_attempted_expansions+1);
        }

        uint number_attempted_expansions{0};
        uint number_successful_expansions{0};

        double parentEdgeWeight{0.0};

        base::State *state{nullptr};
        Configuration *parent{nullptr};
        Configuration *coset{nullptr}; //the underlying coset this configuration belongs to (on the quotient-space)

        bool isSufficientFeasible{false};
        double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
        void *element;
      };

      typedef ompl::PDF<Configuration*> PDF;
      typedef PDF::Element PDF_Element;
      typedef std::shared_ptr<NearestNeighbors<Configuration*>> NearestNeighborsPtr;

      void SetSubGraph( QuotientChart *sibling, uint k ) override;
      bool AddConfiguration(Configuration *q, Configuration *q_parent=nullptr);
      void RemoveConfiguration(Configuration *q);

      bool sampleUniformOnNeighborhoodBoundary(Configuration *sample, const Configuration *center);
      bool sampleHalfBallOnNeighborhoodBoundary(Configuration *sample, const Configuration *center);
      Configuration* EstimateBestNextState(Configuration *q_last, Configuration *q_current);

      virtual bool Sample(Configuration *q_random);
      virtual Configuration* SampleTree(ob::State*);
      bool IsSampleInsideCover(Configuration *q);
      void RemoveCoveredSamples(Configuration *q);

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
      NearestNeighborsPtr cover_tree;
      NearestNeighborsPtr vertex_tree;
      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};
      PDF pdf_necessary_vertices;
      PDF pdf_all_vertices;

      double threshold_clearance{0.01};

    public:

      std::shared_ptr<NearestNeighbors<Configuration *>> GetTree() const;
      Configuration* GetStartConfiguration() const;
      Configuration* GetGoalConfiguration() const;
      const PDF& GetPDFNecessaryVertices() const;
      const PDF& GetPDFAllVertices() const;
      double GetGoalBias() const;
      void freeTree( NearestNeighborsPtr nn);


      //Configuration *lastSampled{nullptr};

    };
  }
}

