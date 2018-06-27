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
    //LH  : linear homotopy
    //QSLH: Quotient-space linear homotopy planner
    class QSLH: public og::QMP{
      private: typedef og::QMP Base;

      public:

        QSLH(const ob::SpaceInformationPtr &si, Quotient *previous_);
        virtual ~QSLH() override;

        virtual void CheckForSolution(ob::PathPtr &solution) override;
        virtual void Init() override;
        virtual void Grow(double t) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;
        virtual bool SampleGraph(ob::State *q_random_graph) override;

      protected:
        ompl::RNG rng;
        double totalGrowTime{0.0};
        double decay_constant{1.0}; //how long should the algorithm try to sample the shortest path until it decides that the path is likely misleading
    };

  };
};
