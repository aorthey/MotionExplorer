#pragma once

#include <ompl/base/Planner.h>

namespace ob = ompl::base;

//  Visualization of CSpace Decomposition
//
// [    ][    ]
// [    ][    ]
// [    ][ M0 ]
// [ M1 ][____]
// [    ][    ]
// [    ][ C1 ]
// [    ][    ]
//
// whereby 
// M1 = M1
// M0 = previous->getspaceinformation()
// C1 = C1
//
//
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
//
//TO be done:
///// M1 SE3      , M0 R3xSO2xSO2         =>C1 = SO2
///// M1 R3xS1xS1 , M0 R3                 =>C1 = SO2xSO2
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

        friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::Quotient& qtnt);

      protected:
        static uint counter;
        uint id;

        enum QuotientSpaceType{ UNKNOWN, RN_RM, SE2_R2, SE3_R3, SE3RN_SE3, SE3RN_SE3RM };

        QuotientSpaceType type;
        uint M1_dimension;
        uint M0_dimension;
        uint C1_dimension;

        const ob::StateSpacePtr ComputeQuotientSpace(const ob::StateSpacePtr M1, const ob::StateSpacePtr M0);
        void mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1);
        void ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const;
        void ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const;

        ob::SpaceInformationPtr M1; //full configuration space Mi = si_
        ob::SpaceInformationPtr C1; //configuration space Ci = Mi/Mi-1
        ob::StateSamplerPtr C1_sampler;
        ob::ValidStateSamplerPtr sampler_;
        ob::StateSamplerPtr simpleSampler_;

        Quotient *previous{nullptr};

    };
  }
}


