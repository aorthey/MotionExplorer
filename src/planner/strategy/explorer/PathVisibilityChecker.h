#pragma once
#include "planner/strategy/quotientgraph/quotient_graph.h"
#include <ompl/base/State.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class PathVisibilityChecker
    {
    public:

      PathVisibilityChecker(const ob::SpaceInformationPtr &si);
      ~PathVisibilityChecker(void);
      // bool IsPathVisible(std::vector<QuotientGraph::Vertex> &v1, std::vector<QuotientGraph::Vertex> &v2);
      bool IsPathVisible(std::vector<QuotientGraph::Vertex> &v1, std::vector<QuotientGraph::Vertex> &v2, QuotientGraph::Graph &graph);
      bool IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
    protected:
      ob::SpaceInformationPtr si_;
    };
  }
}
