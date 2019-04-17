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

void DecompositionPlanner::clear()
{
  BaseT::clear();
}

void DecompositionPlanner::setup(void)
{
  BaseT::setup();
}

void DecompositionPlanner::Grow(double t)
{
  std::cout << "NYI" << std::endl;
}

PlannerDataVertexAnnotated DecompositionPlanner::getAnnotatedVertex(const Vertex &vertex) const
{
  ob::State *state = graph[vertex]->state;
  if(!state){
    std::cout << "vertex state does not exists" << std::endl;
    Q1->printState(state);
    exit(0);
  }

  PlannerDataVertexAnnotated pvertex(state);
  pvertex.SetLevel(GetLevel());
  pvertex.SetPath(GetChartPath());

  return pvertex;
}

void DecompositionPlanner::getPlannerDataAnnotated(base::PlannerData &data) const
{
  PlannerDataVertexAnnotated pstart = getAnnotatedVertex(v_start);
  data.addStartVertex(pstart);

  if(isConnected){
    PlannerDataVertexAnnotated pgoal = getAnnotatedVertex(v_goal);
    data.addGoalVertex(pgoal);
  }

  foreach( const Vertex v, boost::vertices(graph))
  {
    PlannerDataVertexAnnotated p = getAnnotatedVertex(v);
    data.addVertex(p);
  }
  // foreach (const Edge e, boost::edges(graph))
  // {
  //   const Vertex v1 = boost::source(e, graph);
  //   const Vertex v2 = boost::target(e, graph);

  //   const ob::State *s1 = indexToStates[graph[v1]->index];
  //   const ob::State *s2 = indexToStates[graph[v2]->index];
  //   PlannerDataVertexAnnotated p1(s1);
  //   PlannerDataVertexAnnotated p2(s2);
  //   data.addEdge(p1,p2);
  // }
  // if(verbose>0) std::cout << "added " << data.numVertices() << " vertices and " << data.numEdges() << " edges."<< std::endl;
}
