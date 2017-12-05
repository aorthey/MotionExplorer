#pragma once

#include "prm_basic.h"
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <KrisLibrary/math/infnan.h> //dInf, fInf, IsNaN(x)
using Math::dInf;

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
    //class PRMSlice: public og::PRM{
    class PRMSlice: public og::PRMBasic{

      public:
        PRMSlice(const ob::SpaceInformationPtr &si);

        ~PRMSlice() override;

        void Grow(double t);

        void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState);

        virtual bool Sample(ob::State *workState);

        virtual ob::PlannerStatus Init();

        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        void checkForSolution(base::PathPtr &solution);

        bool hasSolution();

    protected:

        std::vector<ob::State *> xstates;

    };

  };
};


