#pragma once
//#include "prm_basic.h"
#include "qmp_connect.h"
#include "planner/cover/open_set_convex.h"
#include "planner/cover/cover.h"
#include "planner/cover/cover_convex_partition.h"
#include "elements/geometry/convex_polyhedron.h"
#include <ompl/datastructures/PDF.h>

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
    //SC  : simplicial complex
    //QSCP: Quotient-space simplicial Complex Planner
    //
    // QSCP works like QMPConnect, but changes the first quotient space to use a
    // volumetric workspace decomposition from which we sample (instead of using
    // a roadmap of the free workspace).
    class QSCP: public og::QMP{
      private: typedef og::QMP Base;

      public:

        QSCP(const ob::SpaceInformationPtr &si, Quotient *previous_);
        virtual ~QSCP() override;

        virtual double GetSamplingDensity() override;
        virtual void Init() override;
        virtual void Grow(double t) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;
        virtual bool SampleGraph(ob::State *q_random_graph) override;

      protected:
        //cover::CoverConvexPartition cspace_cover;
        std::vector<cover::OpenSetConvex*> workspace_regions;
        ompl::RNG rng;
        double totalGrowTime{0.0};
        double decay_constant{1.0}; //how long should the algorithm try to sample the shortest path until it decides that the path is likely misleading
    };

  };
};
