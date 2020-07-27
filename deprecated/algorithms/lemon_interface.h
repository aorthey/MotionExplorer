#pragma once

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

namespace ob = ompl::base;

class LemonInterface{
  typedef lemon::ListGraph::EdgeMap<double> CostMap;
  public:
    LemonInterface( ob::PlannerDataPtr pd_);

    std::vector<ob::PlannerData::Graph::Vertex> GetShortestPath();
    std::vector<ob::PlannerData::Graph::Vertex> GetShortestPath( lemon::ListGraph::Node s, lemon::ListGraph::Node t);

  private:

    ob::PlannerDataPtr pd;
    uint N;

    std::vector<ob::PlannerData::Graph::Vertex> shortest_path_idxs;

    lemon::ListGraph lg;
    std::vector<lemon::ListGraph::Node> gn;
    lemon::ListGraph::Node start, goal;
    bool hasStart, hasGoal;

    CostMap* length;

};

