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
  q_random = new Configuration(Q1);
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
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);
  goal = pdef_->getGoal().get();
}

void DecompositionPlanner::Grow(double t)
{
  if(firstRun){
    Init();
    firstRun=false;
  }
  if(hasSolution){
    //No Goal Biasing if we already found a solution on this quotient space
    Sample(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      Q1->copyState(q_random->state, s_goal);
    }else{
      Sample(q_random->state);
    }
  }

  Configuration *q_nearest = Nearest(q_random);

  double d = Q1->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  if(Q1->checkMotion(q_nearest->state, q_random->state))
  {
    Vertex v = AddConfiguration(q_random->state);
    Configuration *q_next = graph[v];

    AddEdge(q_nearest, q_next);

    double dist = 0.0;
    bool satisfied = goal->isSatisfied(q_next->state, &dist);
    if(satisfied)
    {
      v_goal = AddConfiguration(s_goal);
      AddEdge(q_nearest, graph[v_goal]);
      hasSolution = true;
    }
  }
}

