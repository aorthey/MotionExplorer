#pragma once

#include "planner/cspace.h"
#include "elements/path_pwl_euclid.h"

#include <ompl/base/Cost.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>

#include <list>
#include <utility>
#include <map>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
//#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Cost = ob::Cost;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;
using Edge = Graph::Edge;
using VIterator = Graph::VIterator;
using EIterator = Graph::EIterator;

class OnetopicPathSpaceModifier{
  public:
    explicit OnetopicPathSpaceModifier( ob::PlannerDataPtr pd_in, CSpaceOMPL *cspace);

    std::vector< std::vector< Config >> GetShortestPathForEachCover();

    std::vector< Config> GetAllVertices();
    std::vector< std::pair<Config,Config>> GetAllEdges();
    std::vector< std::vector<Config>> GetAllPaths();

    std::vector< std::vector< Config >> GetCoverVertices();
    std::vector< std::vector< std::pair<Config,Config> >> GetCoverEdges();
    std::vector< std::vector<std::vector<Config> >> GetCoverPaths();

  private:

    void ComputeShortestPathsLemon();
    void ShortestPathsNonOnetopic();
    void ComputeOnetopicCoverRoadmap();
    std::vector<Config> VertexPathToConfigPath( const std::vector<Vertex> &path);
    Config VertexToConfig( const Vertex& v);
  private:

    ob::PlannerDataPtr pd;

    CSpaceOMPL *cspace;

    std::vector< Vertex > all_vertices;
    std::vector< Config > all_vertices_config;

    std::vector< std::pair<Vertex,Vertex> > all_edges;
    std::vector< std::pair<Config,Config> > all_edges_config;

    std::vector< std::vector<Vertex>> all_paths;
    std::vector< std::vector<Config>> all_paths_config;


    std::vector<double> distance_shortest_path; //length of the shortest path from start to goal >including< vertex i
    double max_distance_shortest_path;
    double min_distance_shortest_path;


    std::vector< std::vector< Vertex >> cover_vertices;
    std::vector< std::vector< Config >> cover_vertices_config;

    std::vector< std::vector< std::pair<Vertex,Vertex> >> cover_edges;
    std::vector< std::vector< std::pair<Config,Config> >> cover_edges_config;

    std::vector< std::vector<std::vector<Vertex> >> cover_paths;
    std::vector< std::vector<std::vector<Config> >> cover_paths_config;


    std::vector< std::vector< Vertex >> cover_shortest_paths_vertex;
    std::vector< std::vector< Config >> cover_shortest_paths_config;
    std::vector< std::vector< const ob::State* >> cover_shortest_paths_omplstate;

};
