#pragma once

#include <ompl/base/PlannerData.h>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
namespace ob = ompl::base;

class LemonInterface{
  typedef lemon::ListGraph::EdgeMap<double> CostMap;
  public:
    LemonInterface( ob::PlannerDataPtr pd_ );

    std::vector<Config> GetShortestPath( int s, int t);
    std::vector<int> GetShortestPathIndex( int s, int t);

  private:
    ob::PlannerDataPtr pd;
    uint N;

    lemon::ListGraph lg;
    std::vector<lemon::ListGraph::Node> gn;
    lemon::ListGraph::Node start, goal;

    CostMap* length;

};

