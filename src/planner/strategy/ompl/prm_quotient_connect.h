#pragma once
#include "prm_quotient.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace base
  {
    OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class PRMQuotientConnect: public og::PRMQuotient{

      public:

        PRMQuotientConnect(const ob::SpaceInformationPtr &si, Quotient *previous_);

        ~PRMQuotientConnect() override;

        ob::PathPtr GetShortestPathOffsetVertices( const ob::State *qa, const ob::State *qb, 
          const Vertex vsa, const Vertex vsb, const Vertex vta, const Vertex vtb);

        ob::PathPtr InterpolateM1GraphConstraint( const Vertex a, const Vertex b) const;

        void setup() override;
        virtual void Init() override;

      protected:

        virtual bool SampleGraph(ob::State*) override;
        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual bool Connect(const Vertex a, const Vertex b) override;
        virtual bool Sample(ob::State *q_random) override;
        virtual Vertex CreateNewVertex(ob::State *state) override;
        virtual ompl::PDF<og::PRMBasic::Edge> GetEdgePDF() override;

        virtual void RandomWalk(const Vertex &v) override;


      public:
        double percentageSamplesOnShortestPath{0.2};
        double goalBias_{0.05};
        int lastSourceVertexSampled{-1};
        int lastTargetVertexSampled{-1};
        double lastTSampled{-1.0};
        bool isSampled{false};
    };

  };
};

