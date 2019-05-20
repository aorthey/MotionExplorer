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

//https://de.wikipedia.org/wiki/Strategie_(Entwurfsmuster)
bool DecompositionPlanner::IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2)
{
  bool connectionPossible = true;
  for(uint k = 0; k < min(s1.size(),s2.size()); k++) {
    if(!si_->checkMotion(s1.at(k), s2.at(k))){
      connectionPossible = false;
      break;
    }
  }
  return connectionPossible;
}


//############################################################################
//Some Useful Function for PathVisibility Computation:
//############################################################################
//typedef std::vector<Vertex> Path;
////(0) How to get start and goal states
//ob::State *s_start = G[v_start]->state;
//ob::State *s_goal = G[v_goal]->state;

////(1) How to get an ompl::base::State* from the vertex v
//ob::State *sv = G[v]->state;

////(2) How to interpolate between two states
//ob::State *s_interpolated = si_->allocState();
//si_->getStateSpace()->interpolate(s_start, s_goal, 0.5, s_interpolated);

////(2) How to Iterate through subgraph G
//foreach( const Vertex vg, boost::vertices(G))
//{
//  ob::State *sg = G[vg]->state;
//  si_->getStateSpace()->interpolate(sv, sg, 0.2, s_interpolated);
//}

////(3) How to get shortest path
////  v_start and v_goal are start and goal vertex (from quotientchartsubgraph)
//Path p_start_goal = GetPathOnGraph(v_start, v_goal);
//Path p_v = GetPathOnGraph(v_start, v, v_goal);

////(4) How to check if linear path between two states is feasible
//if(si_->checkMotion(s_start,s_goal))
//{
//  std::cout << "Feasible Path exists between states" << G[v_start] << " and " << G[v_goal] << std::endl;
//}
//############################################################################

