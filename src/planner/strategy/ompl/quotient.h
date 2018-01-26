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


