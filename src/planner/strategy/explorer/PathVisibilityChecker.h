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
      bool IsPathVisibleSO2(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
      bool CheckValidity(const std::vector<ob::State*> &s);

      void Test1();
      void Test2();
      void Test3(int F=0);
    protected:
      ob::SpaceInformationPtr si_;
      ob::State *lastValidState;
    private:
      std::vector<ob::State*> StatesFromVector( 
          const std::vector<double> &sx, 
          const std::vector<double> &sy);
      std::vector<ob::State*> StatesFromVectorSO2R1( 
          const std::vector<double> &st, 
          const std::vector<double> &sx);
      std::vector<ob::State*> StatesFromVector( 
          const std::vector<double> &sx, 
          const std::vector<double> &sy, 
          const std::vector<double> &st);
    };
  }
}
