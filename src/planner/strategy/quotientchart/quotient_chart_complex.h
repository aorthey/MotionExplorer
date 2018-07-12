#pragma once
#include "quotient_chart.h"
#include <gudhi/Simplex_tree.h>

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class QuotientChartComplex: public QuotientChart
    {
        typedef og::QuotientChart BaseT;
        typedef og::QuotientGraph::Graph Graph;
        using Simplex_tree = Gudhi::Simplex_tree<>;
      public:
        QuotientChartComplex(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);
        virtual void setup() override;

        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual void Grow(double t) override;
        virtual bool Sample(ob::State *q_random) override;
        virtual void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;

      private:
        double epsilon_max_neighborhood{2.0};
        Simplex_tree simplex;

        uint ntry;

        typedef std::vector< std::vector<int> > LocalSimplicialComplex;
        std::map<const ob::State*, LocalSimplicialComplex> simplicial_complex;

        Graph G_infeasible;
        RoadmapNeighbors nn_infeasible;
    };
  }
}
