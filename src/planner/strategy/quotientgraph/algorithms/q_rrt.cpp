#include "q_rrt.h"
#include "common.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

#include <ompl/datastructures/PDF.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QRRT::QRRT(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
  setName("QRRT"+std::to_string(id));
}

QRRT::~QRRT()
{
}

bool QRRT::GetSolution(ob::PathPtr &solution)
{
  if(hasSolution){
    return BaseT::GetSolution(solution);
  }else{
    const Configuration *q_nearest = Nearest(q_goal);
    double d = Q1->distance(q_nearest->state, q_goal->state);
    if(d < epsilon)
    {
      v_goal = AddConfiguration(q_goal);
      AddEdge(q_nearest->index, v_goal);
      solution_path = GetPath(v_start, v_goal);
      hasSolution = true;
    } 
  }
  return hasSolution;
}


void QRRT::Grow(double t){
  if(firstRun){
    Init();
    firstRun = false;
    q_random = new Configuration(Q1);
  }

  double s = rng_.uniform01();
  if(s < goalBias_){
    Q1->copyState(q_random->state, q_goal->state);
  }else{
    Sample(q_random->state);
  }

  const Configuration *q_nearest = Nearest(q_random);
  double d = Q1->distance(q_nearest->state, q_random->state);
  Configuration *q_next = new Configuration(Q1, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_next->state);
  }

  if(Q1->checkMotion(q_nearest->state, q_next->state))
  {
    Vertex v_next = AddConfiguration(q_next);
    AddEdge(q_nearest->index, v_next);
  }
}

bool QRRT::SampleQuotient(ob::State *q_random_graph)
{
  const Vertex v = boost::random_vertex(G, rng_boost);
  Q1->getStateSpace()->copyState(q_random_graph, G[v]->state);
  Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);
  return true;
}
