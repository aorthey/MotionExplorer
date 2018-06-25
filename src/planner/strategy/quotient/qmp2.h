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
    class QMP2: public og::QuotientGraph{

      public:

        QMP2(const ob::SpaceInformationPtr &si, Quotient *previous_);
        virtual ~QMP2() override;

      protected:
        double epsilon{0.01}; //graph thickening
        double percentageSamplesOnShortestPath{0.1};

        virtual bool SampleGraph(ob::State*) override;
        virtual ompl::PDF<og::QuotientGraph::Edge> GetEdgePDF();
        uint samplesOnShortestPath{0};

    };

  };
};
