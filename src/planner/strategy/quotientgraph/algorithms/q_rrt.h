#pragma once
#include "planner/strategy/quotientgraph/quotient_graph.h"
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
    //QRRT: Quotient-space roadMap Planner
    class QRRT: public og::QuotientGraph{

      typedef og::QuotientGraph BaseT;
      public:

        QRRT(const ob::SpaceInformationPtr &si, Quotient *parent_);
        virtual ~QRRT() override;
        virtual void Grow(double t) override;
        virtual bool GetSolution(ob::PathPtr &solution) override;
        double GetImportance() const override;

      protected:

        double maxDistance{1.0};
        double epsilon{0.0};
        double goalBias_{0.05};
        PDF<Vertex> vpdf;
        PDF<Edge> pdf_edges_on_shortest_path;
        Configuration *q_random{nullptr};

        virtual bool SampleQuotient(ob::State*) override;

    };

  };
};
