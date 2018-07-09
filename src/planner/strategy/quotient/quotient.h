#pragma once

#include <ompl/base/Planner.h>

namespace ob = ompl::base;

//  Quotient contains a single Quotient Space Decomposition which is nested in a chain of
//  quotient spaces. This single decomposition can be visualized as
//
// [    ][ M0 ]
// [ M1 ][____]
// [    ][ C1 ]
// [    ][    ]
//
//  whereby M1 is the configuration space, C1 is a designated subspace of M1, and M0 =
//  M1/C1 is the resulting quotient space. Here we assume that M1 and M0 have
//  been given, and we compute the inverse of the quotient map, i.e. C1 = M1/M0. 
//
//Cases we can handle:
// ---- non-compound:
// (1) M1 Rn       , M0 Rm       , 0<m<n   => C1 = R(n-m)
// ---- compound:
// (2) M1 SE2      , M0 R2                 => C1 = SO2
// (3) M1 SE3      , M0 R3                 => C1 = SO3
// (4) M1 SE3xRn   , M0 SE3                => C1 = Rn
// (5) M1 SE3xRn   , M0 SE3xRm   , 0<m<n   => C1 = R(n-m)
//
//TO be done (might be beneficial for rigid objects floating in space)
///// M1 SE3      , M0 R3xSO2xSO2         =>C1 = SO2
///// M1 R3xS1xS1 , M0 R3                 =>C1 = SO2xSO2
namespace ompl
{
  namespace geometric
  {
    class Quotient: public ob::Planner
    {
      public:
        uint verbose = 0;
        Quotient(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);
        virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

        virtual bool Sample(ob::State *q_random);

        virtual void Init() = 0;
        virtual void Grow(double t) = 0;
        virtual void CheckForSolution(ob::PathPtr &solution) = 0;
        virtual bool SampleGraph(ob::State *q_random) = 0;

        virtual double GetSamplingDensity();
        virtual uint GetNumberOfVertices() const;
        virtual uint GetNumberOfEdges() const;

        virtual bool HasSolution();
        virtual uint GetNumberOfSampledVertices();
        static void resetCounter();

        friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::Quotient& qtnt);

        const ob::SpaceInformationPtr &getC1() const;
        bool SampleC1(ob::State *s);
        void mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1) const;

        Quotient* GetParent() const;
        Quotient* GetChild() const;
        void SetChild(Quotient *child_);
        void SetParent(Quotient *parent_);

        double GetGraphLength();
        virtual void clear() override;

      protected:
        static uint counter;
        uint id;

        enum QuotientSpaceType{ UNKNOWN, ATOMIC_RN, RN_RM, SE2_R2, SE3_R3, SE3RN_R3, SE3RN_SE3, SE3RN_SE3RM };

        QuotientSpaceType type;
        uint M1_dimension;
        uint M0_dimension;
        uint C1_dimension;

        const ob::StateSpacePtr ComputeQuotientSpace(const ob::StateSpacePtr M1, const ob::StateSpacePtr M0);
        void ExtractC1Subspace( const ob::State* q, ob::State* qC1 ) const;
        void ExtractM0Subspace( const ob::State* q, ob::State* qM0 ) const;

        ob::SpaceInformationPtr M1; //configuration space Mi = si_
        ob::SpaceInformationPtr M0; //quotient space Mi-1 = Mi/Ci
        ob::SpaceInformationPtr C1; //subspace Ci = Mi/Mi-1

        ob::StateSamplerPtr C1_sampler;
        ob::StateSamplerPtr M1_sampler;
        ob::ValidStateSamplerPtr M1_valid_sampler;

        double graphLength{0.0};
        uint totalNumberOfSamples{0};
        bool hasSolution{false};

        Quotient *parent{nullptr};
        Quotient *child{nullptr};
    };
  }
}


