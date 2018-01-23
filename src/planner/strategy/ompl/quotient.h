#pragma once

#include <ompl/base/Planner.h>

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class Quotient: public ob::Planner
    {
      public:
        Quotient(const ob::SpaceInformationPtr &si, Quotient *previous_ = nullptr);
        virtual bool Sample(ob::State *q_random);
        virtual bool SampleGraph(ob::State *q_random);

        virtual void Init() = 0;
        virtual void Grow(double t) = 0;
        virtual bool HasSolution() = 0;
        virtual void CheckForSolution(ob::PathPtr &solution) = 0;
        double GetSamplingDensity();
        virtual uint GetNumberOfVertices() = 0;

        static void resetCounter();
      protected:
        static uint counter;
        uint id;

        void mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1);
        void ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const;
        void ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const;

        ob::SpaceInformationPtr M1; //full configuration space Mi = si_
        ob::SpaceInformationPtr C1; //configuration space Ci = Mi/Mi-1
        ob::StateSamplerPtr C1_sampler;
        ob::ValidStateSamplerPtr sampler_;
        ob::StateSamplerPtr simpleSampler_;

        uint M0_subspaces;
        uint M1_subspaces;
        uint C1_subspaces;

        Quotient *previous{nullptr};

    };
  }
}

