#pragma once
#include "quotient_chart.h"

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class QuotientChartComplex: public QuotientChart
    {
        typedef og::QuotientChart BaseT;
        typedef og::QuotientGraph::Graph Graph;
      public:
        QuotientChartComplex(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);

        virtual bool Sample(ob::State *q_random) override;
        virtual void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *) override;

      private:
        Graph G_infeasible;
        RoadmapNeighbors nn_infeasible;
    };
  }
}
