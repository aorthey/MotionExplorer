#include "q_rrt.h"
#include "common.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

#include <ompl/tools/config/SelfConfig.h>
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
  Planner::declareParam<double>("range", this, &QRRT::setRange, &QRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &QRRT::setGoalBias, &QRRT::getGoalBias, "0.:.05:1.");
  q_random = new Configuration(Q1);
}

QRRT::~QRRT()
{
  DeleteConfiguration(q_random);
}

void QRRT::setGoalBias(double goalBias_)
{
  goalBias = goalBias_;
}
double QRRT::getGoalBias() const
{
  return goalBias;
}
void QRRT::setRange(double maxDistance_)
{
  maxDistance = maxDistance_;
}
double QRRT::getRange() const
{
  return maxDistance;
}

void QRRT::setup()
{
  BaseT::setup();
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);
  //const double base = 2;
  //const double normalizer = powf(base, level);
  //epsilon = 0.1/normalizer;

  goal = pdef_->getGoal().get();
}
void QRRT::clear()
{
  BaseT::clear();
}

bool QRRT::GetSolution(ob::PathPtr &solution)
{
  if(hasSolution){
    return BaseT::GetSolution(solution);
  }else{
    if(firstRun) return false;

    const Configuration *q_nearest = Nearest(q_goal);
    //double d = Q1->distance(q_nearest->state, q_goal->state);
    double dist = 0.0;
    bool satisfied = goal->isSatisfied(q_nearest->state, &dist);
    if(satisfied)
    {
      v_goal = AddConfiguration(q_goal);
      AddEdge(q_nearest->index, v_goal);
      solution_path = GetPath(v_start, v_goal);
      hasSolution = true;
    }
    // if(d < 0.01)
    // {
    //   v_goal = AddConfiguration(q_goal);
    //   AddEdge(q_nearest->index, v_goal);
    //   solution_path = GetPath(v_start, v_goal);
    //   hasSolution = true;
    // } 
  }
  return hasSolution;
}


void QRRT::Grow(double t){
  if(firstRun){
    Init();
    firstRun = false;
  }

  if(hasSolution){
    //No Goal Biasing if we already found a solution on this quotient space
    totalNumberOfSamples++;
    Sample(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      Q1->copyState(q_random->state, q_goal->state);
      totalNumberOfSamples++;
    }else{
      Sample(q_random->state);
    }
  }

  const Configuration *q_nearest = Nearest(q_random);
  double d = Q1->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  if(Q1->checkMotion(q_nearest->state, q_random->state))
  {
    Configuration *q_next = new Configuration(Q1, q_random->state);
    Vertex v_next = AddConfiguration(q_next);
    AddEdge(q_nearest->index, v_next);
  }
}

double QRRT::GetImportance() const{
  //Should depend on
  // (1) level : The higher the level, the more importance
  // (2) total samples: the more we already sampled, the less important it
  // becomes
  // (3) has solution: if it already has a solution, we should explore less
  // (only when nothing happens on other levels)
  // (4) vertices: the more vertices we have, the less important (let other
  // levels also explore)
  //
  //
  //  exponentially more samples on level i. Should depend on ALL levels.
  const double base = 2;
  const double normalizer = powf(base, level);
  double N = (double)GetNumberOfVertices()/normalizer;
  return 1.0/(N+1);
}

bool QRRT::SampleQuotient(ob::State *q_random_graph)
{
  //VERTEX SAMPLING
  const Vertex v = boost::random_vertex(G, rng_boost);
  Q1->getStateSpace()->copyState(q_random_graph, G[v]->state);
  //if(epsilon > 0) Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);

  //EDGE SAMPLING (biased towards highly connected vertices, which might be a
  //good thing)
  // if(num_edges(G) == 0) return false;

  // Edge e = boost::random_edge(G, rng_boost);
  // double s = rng_.uniform01();
  // const Vertex v1 = boost::source(e, G);
  // const Vertex v2 = boost::target(e, G);
  // const ob::State *from = G[v1]->state;
  // const ob::State *to = G[v2]->state;

  // Q1->getStateSpace()->interpolate(from, to, s, q_random_graph);
  return true;
}
