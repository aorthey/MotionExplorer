#pragma once
#include "quotient_chart_complex.h"
#include <ompl/datastructures/PDF.h>
#include <boost/graph/random.hpp> 
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
  namespace geometric
  {
    //QCP: Quotient-space roadMap Planner
    class QCP: public og::QuotientChartComplex{
      typedef og::QuotientChartComplex BaseT;
      public:

        QCP(const ob::SpaceInformationPtr &si, QuotientChart *parent_ = nullptr);
        virtual ~QCP() override;

      protected:
        typedef boost::minstd_rand RNGType;
        RNGType rng;

        double epsilon{0.0}; //graph thickening
        double percentageSamplesOnShortestPath{0.0};
        double goalBias_{0.0};
        PDF<Vertex> vpdf;

        virtual bool SampleGraph(ob::State*) override;
        virtual ompl::PDF<og::QuotientChart::Edge> GetEdgePDF();
        uint samplesOnShortestPath{0};

    };

  };
};
