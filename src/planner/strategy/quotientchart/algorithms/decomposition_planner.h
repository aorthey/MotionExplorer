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
      virtual bool IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2) override;
    };
  }
}

