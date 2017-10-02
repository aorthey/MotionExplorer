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
    explicit OnetopicPathSpaceModifier( ob::PlannerData& pd_in, CSpaceOMPL *cspace);

    std::vector< std::vector< Config >> GetConfigPaths();
    std::vector< std::vector< const ob::State* >> GetOMPLStatePaths();

  private:
    void ComputeShortestPathsLemon(ob::PlannerData& pd_in);
    void InterpolatePaths( ob::PlannerData& pd );

    std::vector< std::vector< Config >> config_paths;
    std::vector< std::vector< const ob::State* >> omplstate_paths;

    CSpaceOMPL *cspace;

};
