#pragma once
#include "planner/cspace/cspace.h"
#include "elements/path_pwl.h"

#include <ompl/base/Cost.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Graph = ob::PlannerData::Graph;
using Vertex = Graph::Vertex;

//check if two paths p1,p2 \in P are visible.
//
// given: roadmap of vertices and edges. 
// p1 is defined as a set of vertices.
//
// return true/false
//

static ob::OptimizationObjectivePtr getThresholdPathLength(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(dInf));
  return obj;
}

class VisibilityChecker{

  const double max_planning_time_path_path=0.05;
  const double max_planning_time_path_vertex=0.03;
  const double epsilon_goalregion = 0.01;

  public:
    VisibilityChecker(const ob::SpaceInformationPtr si_path_, CSpaceOMPL *cspace_);

    //path <-> path visibility
    bool isVisiblePathPath(const ob::PlannerData& pd, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2);
    //path <-> vertex visibility
    bool isVisiblePathVertex(const ob::PlannerData& pd, const std::vector<Vertex> &p1, const Vertex& vertex);
    //vertex <-> vertex visibility
    bool isVisibleVertexVertex(const ob::PlannerData& pd, const Vertex& v1, const Vertex& v2);

  private:
    ob::SpaceInformationPtr si_path;
    CSpaceOMPL *cspace;
};
