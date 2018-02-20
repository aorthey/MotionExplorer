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

        void setup() override;
        virtual void Init() override;

      protected:

        virtual bool SampleGraph(ob::State*) override;
        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual bool Connect(const Vertex a, const Vertex b) override;

        virtual uint randomBounceMotion(const ob::StateSamplerPtr &sss, 
          const Vertex &v, std::vector<ob::State *> &states) const override;

        virtual Vertex addMilestone(ob::State *state) override;

        uint lastSourceVertexSampled;
        uint lastTargetVertexSampled;
        double lastTSampled;
        bool isSampled{false};
    };

  };
};

