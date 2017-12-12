#pragma once
#include "prm_basic.h"

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
    class PRMSliceConnect: public og::PRMBasic{

      public:

        PRMSliceConnect(const ob::SpaceInformationPtr &si, PRMSliceConnect *previous_);

        ~PRMSliceConnect() override;

        void getPlannerData(base::PlannerData &data) const override;

        double getSamplingDensity();

        virtual ob::PlannerStatus Init();

        void setup() override;

        Vertex lastSourceVertexSampled;

        Vertex lastTargetVertexSampled;

        double lastTSampled;

        bool isSampled{false};

        static void resetCounter();

      protected:

        static uint counter;
        uint id;

        //Overrides Distance/Sample/Connect
        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual bool Sample(ob::State *workState) override;
        virtual bool Connect(const Vertex a, const Vertex b) override;

        bool SampleGraph(ob::State *workState);

        ob::PathPtr GetShortestPathOffsetVertices( const ob::State *qa, const ob::State *qb, 
          const Vertex vsa, const Vertex vsb, const Vertex vta, const Vertex vtb);

        virtual Vertex addMilestone(ob::State *state) override;

        void mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1);
        void ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const;
        void ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const;

        ob::SpaceInformationPtr M1; //full configuration space Mi = si_
        ob::SpaceInformationPtr C1; //configuration space Ci = Mi/Mi-1

        base::StateSamplerPtr C1_sampler;

        uint M0_subspaces;
        uint M1_subspaces;
        uint C1_subspaces;

        PRMSliceConnect *previous;

    };

  };
};

