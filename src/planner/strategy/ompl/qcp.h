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
    //QSCP: Quotient-space simplicial Complex Planner
    //SC : simplicial complex
    //
    class QCP: public og::QMPConnect{
      typedef og::QMPConnect Base;

      public:

        QCP(const ob::SpaceInformationPtr &si, Quotient *previous_);
        virtual ~QCP() override;

        virtual double GetSamplingDensity() override;
        virtual void Init() override;
        virtual void Grow(double t) override;
        virtual void CheckForSolution(ob::PathPtr &solution) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;
        virtual bool SampleGraph(ob::State *q_random_graph) override;

      protected:
        //cover::CoverConvexPartition cspace_cover;
        std::vector<cover::OpenSetConvex*> workspace_regions;
        ompl::RNG rng;
    };

  };
};
