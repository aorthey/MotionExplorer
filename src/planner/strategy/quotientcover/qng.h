#pragma once
#include "planner/strategy/quotientcover/quotient_cover.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //Quotient-space sufficient Neighborhood Graph planner (QNG)
    class QNG: public og::QuotientChartCover
    {
      typedef og::QuotientChartCover BaseT;
    public:

      uint verbose{0};

      QNG(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNG(void);
      virtual void Grow(double t) override;
      virtual void setup() override;
      Configuration* EstimateBestNextState(Configuration *q_last, Configuration *q_current);

      virtual Configuration* SampleCoverBoundary() override;

      void AddToFastTrackConditional(std::vector<Configuration*> q_vector);
      std::vector<Configuration*> ExpandNeighborhood(Configuration *q);

      double importanceSamplingBias{0.5};
      uint NUMBER_OF_EXPANSION_SAMPLES{3};

    private:
      //fast track configurations are boundary nodes which are locally the
      //largest, i.e. for a given neighborhood, if we have N samples on the
      //boundary, then the fast track configuration is the samples with the
      //largest neighborhood. we like to expand them first, since they can lead
      //us to free open spaces and help us navigate through narrow passages
      std::vector<Configuration*> fast_track_configurations;
    };
  }
}

