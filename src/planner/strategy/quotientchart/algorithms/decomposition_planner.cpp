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

  //typedef std::vector<Vertex> Path;
  //Path p_start_goal = GetPathOnGraph(v_start, v_goal);
  //Path p = GetPathOnGraph(v_start, v, v_goal);
  //std::cout << "PATH:" << p_start_goal << std::endl;

  //Configuration *q = G[v];
  //ob::State *s1 = q->state;

  ////Iterate through graph
  //foreach( const Vertex vg, boost::vertices(G))
  //{
  //  Configuration *q = G[vg];
  //  ob::State *s2 = q->state;
  //  bool b = si_->checkMotion(s1,s2);
  //  std::cout << b << std::endl;
  //}

  //for(uint i = 0; i < p_start_goal.size(); i++){
  //  Configuration *q = G[p_start_goal.at(i)];
  //  ob::State *si = q->state;
  //  bool b = si_->checkMotion(s1,si);
  //  ob::State *s_interpolated = si_->allocState();
  //  si_->interpolate(s1, si, 0.5, s_interpolated);
  //  std::cout << b << std::endl;
  //}

  std::vector<int> components;
  if(K>0) components.push_back(0);
  return components;
}

