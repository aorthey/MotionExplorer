#pragma once
#include "quotient_graph.h"
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

      public:

        QMP(const ob::SpaceInformationPtr &si, Quotient *previous_);
        virtual ~QMP() override;

      protected:
        double epsilon{0.01}; //graph thickening
        double percentageSamplesOnShortestPath{0.1};
        double goalBias_{0.05};

        virtual bool SampleGraph(ob::State*) override;
        virtual ompl::PDF<og::QuotientGraph::Edge> GetEdgePDF();
        uint samplesOnShortestPath{0};

    };

  };
};
