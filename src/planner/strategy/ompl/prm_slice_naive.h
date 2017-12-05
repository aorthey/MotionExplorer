#pragma once
#include "prm_slice.h"

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
    class PRMSliceNaive: public og::PRMSlice{

      public:


        PRMSliceNaive(const ob::SpaceInformationPtr &si, PRMSliceNaive *previous_);

        ~PRMSliceNaive() override;

        virtual bool Sample(ob::State *workState) override;
        bool SampleGraph(ob::State *workState);

        void getPlannerData(base::PlannerData &data) const override;

        double getSamplingDensity();

        double distanceFunction(const Vertex a, const Vertex b) const;

        virtual ob::PlannerStatus Init() override;
        void setup() override;

        void mergeStates(ob::State *qM0, ob::State *qC1, ob::State *qM1);

        void ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const;
        void ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const;

        //double distanceGraphFunction(ob::State *qa, ob::State *qb);
        double distanceGraphFunction(ob::State *qa, ob::State *qb, const Vertex va, const Vertex vb);

        og::PRMPlain::Vertex lastVertexSampled;
        double lastTSampled;
        bool isSampled{false};

      protected:
        //virtual Vertex addMilestone(base::State *state) override;

        ob::SpaceInformationPtr M1; //full configuration space Mi = si_
        ob::SpaceInformationPtr C1; //configuration space Ci = Mi/Mi-1

        base::StateSamplerPtr C1_sampler;

        uint M0_subspaces;
        uint M1_subspaces;
        uint C1_subspaces;

        PRMSliceNaive *previous;
    };

  };
};
