#pragma once
#include "planner/strategy/quotientchart/quotient_chart_subgraph.h"
#include "planner/strategy/quotientgraph/quotient_graph.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class DecompositionPlanner: public og::QuotientChartSubGraph
    {
      typedef og::QuotientChartSubGraph BaseT;
    public:

      DecompositionPlanner(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~DecompositionPlanner(void);
      virtual void Grow(double t) override;
      virtual bool FoundNewComponent() override;

      virtual SubGraph& GetSubGraphComponent(uint k_component);

      virtual void setup() override;
      virtual void clear() override;
    protected:
      uint numberOfComponents{0};
    };
  }
}

