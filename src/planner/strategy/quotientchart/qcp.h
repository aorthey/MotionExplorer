#pragma once
#include "quotient_chart.h"
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
    class QCP: public og::QuotientChart{
      typedef og::QuotientChart BaseT;
      public:

        QCP(const ob::SpaceInformationPtr &si, QuotientChart *parent_ = nullptr);
        virtual ~QCP() override;

      protected:
        typedef boost::minstd_rand RNGType;
        RNGType rng;

        double epsilon{0.05}; //graph thickening
        double percentageSamplesOnShortestPath{0.8};
        double goalBias_{0.05};
        PDF<Vertex> vpdf;

        virtual bool SampleGraph(ob::State*) override;
        virtual ompl::PDF<og::QuotientChart::Edge> GetEdgePDF();
        uint samplesOnShortestPath{0};

    };

  };
};
