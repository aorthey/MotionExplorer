#pragma once
#include "prm_basic.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

//
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
// Standard PRM is sampling in M1
// PRMSlice is sampling in G0 x C1, whereby G0 is the roadmap on M0
//
//
// Multilevel $M0 \subspace M1 \subspace M2$
//
// [    ][    ][    ]
// [    ][    ][ M0 ]
// [    ][    ][    ]
// [    ][ M1 ][____]
// [    ][    ]
// [ M2 ][    ]
// [    ][    ]
// [    ][____]
// [    ]
// [    ]
// [____]
//
// [    ][    ][    ]
// [    ][    ][ C0 ]
// [    ][    ][    ]
// [    ][ M1 ][____]
// [    ][    ][    ]
// [ M2 ][    ][ C1 ]
// [    ][    ][    ]
// [    ][____][____]
// [    ][    ][    ]
// [    ][ C2 ][ C2 ]
// [____][____][____]


namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class PRMSlice: public og::PRMBasic{

      public:

        PRMSlice(const ob::SpaceInformationPtr &si, PRMSlice *previous_);

        virtual ~PRMSlice() override;

        void getPlannerData(base::PlannerData &data) const override;

        double getSamplingDensity();

        virtual ob::PlannerStatus Init();

        void setup() override;
        void clear() override;

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

        PRMSlice *previous{nullptr};

    };

  };
};
