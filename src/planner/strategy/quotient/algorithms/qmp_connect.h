#pragma once
#include "qmp.h"

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
    class QMPConnect: public og::QMP{

        typedef og::QMP BaseT;
      public:

        QMPConnect(const ob::SpaceInformationPtr &si, Quotient *parent_);

        ~QMPConnect() override;

        ob::PathPtr GetShortestPathOffsetVertices( const ob::State *qa, const ob::State *qb, 
          const Vertex vsa, const Vertex vsb, const Vertex vta, const Vertex vtb);

        ob::PathPtr InterpolateQ1GraphConstraint( const Vertex a, const Vertex b) const;

        void setup() override;
        virtual void clear() override;
        virtual void Init() override;

      protected:

        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual bool Connect(const Vertex a, const Vertex b) override;
        virtual bool Sample(ob::State *q_random) override;
        virtual Vertex CreateNewVertex(ob::State *state) override;

        virtual void RandomWalk(const Vertex &v) override;


      public:
        int lastSourceVertexSampled{-1};
        int lastTargetVertexSampled{-1};
        double lastTSampled{-1.0};
        bool isSampled{false};
    };

  };
};

