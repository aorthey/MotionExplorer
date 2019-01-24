#pragma once
#include "planner/strategy/quotient/quotient_graph.h"
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
    //QMP: Quotient-space roadMap Planner
    class QMP: public og::QuotientGraph{

      typedef og::QuotientGraph BaseT;
      public:

        QMP(const ob::SpaceInformationPtr &si, Quotient *parent_);
        virtual ~QMP() override;

        virtual bool GetSolution(ob::PathPtr &solution) override;
      protected:

        double epsilon{0.05}; //graph thickening
        double percentageSamplesOnShortestPath{1.0};
        double goalBias_{0.1};
        PDF<Vertex> vpdf;

        PDF<Edge> pdf_edges_on_shortest_path;

        virtual bool SampleQuotient(ob::State*) override;
        uint samplesOnShortestPath{0};

    };

  };
};
