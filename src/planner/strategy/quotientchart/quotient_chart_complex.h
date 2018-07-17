#pragma once
#include "quotient_chart.h"
#include "simplicial_complex.h"

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class QuotientChartComplex: public QuotientChart
    {
        typedef og::QuotientChart BaseT;

      public:
        QuotientChartComplex(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);
        virtual void setup() override;

        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual void Grow(double t) override;
        virtual bool Sample(ob::State *q_random) override;
        //virtual void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;

        void AddSimplices(Vertex v1, Vertex v2);
        void RemoveSimplices(Vertex v1, Vertex v2);

      private:
        double epsilon_max_neighborhood{1.0};
        uint ntry;

        SimplicialComplex *simplicial_complex;

    };
  }
}
