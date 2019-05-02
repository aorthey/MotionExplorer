#include "common.h"

#include "decomposition_planner.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

DecompositionPlanner::DecompositionPlanner(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("DecompositionPlanner"+std::to_string(id));
}

DecompositionPlanner::~DecompositionPlanner(void)
{
}

std::vector<int> DecompositionPlanner::VertexBelongsToComponents(const SubGraph &G, const Vertex &v, int K)
{
  //return an int in [0,K-1] if v does belong to any of the K components
  //return -1 if Vertex does not belong to any component

  //Configuration *q = G[v];
  //q->components;

  //typedef std::vector<Vertex> Path;
  //Path p_start_goal = GetPathOnGraph(v_start, v_goal);
  //std::cout << "PATH:" << p_start_goal << std::endl;

  std::vector<int> components;
  if(K>0) components.push_back(0);
  return components;
}

