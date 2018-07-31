#include "qmp.h"
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

QMP::QMP(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
  setName("QMP"+std::to_string(id));
}

QMP::~QMP()
{
  samplesOnShortestPath = 0;
}

bool QMP::SampleGraph(ob::State *q_random_graph)
{
  //PDF<Edge> pdf = GetEdgePDF();
  //Edge e = pdf.sample(rng_.uniform01());

  Edge e;
  double t = rng_.uniform01();
  if(t<percentageSamplesOnShortestPath)
  {
    //shortest path heuristic
    PDF<Edge> pdf;
    //percentageSamplesOnShortestPath = exp(-pow(((double)samplesOnShortestPath++/1000.0),2));

    for(uint k = 0; k < startGoalVertexPath_.size()-1; k++){
      Vertex v1 = startGoalVertexPath_.at(k);
      Vertex v2 = startGoalVertexPath_.at(k+1);
      Edge ek = boost::edge(v1,v2,G).first;
      pdf.add(ek, G[ek].getCost().value());
    }
    e = pdf.sample(rng_.uniform01());
  }else{
    e = boost::random_edge(G, rng_boost);
    while(!sameComponent(boost::source(e, G), startM_.at(0)))
    {
      e = boost::random_edge(G, rng_boost);
    }
  }

  double s = rng_.uniform01();

  const Vertex v1 = boost::source(e, G);
  const Vertex v2 = boost::target(e, G);
  const ob::State *from = G[v1].state;
  const ob::State *to = G[v2].state;

  M1->getStateSpace()->interpolate(from, to, s, q_random_graph);
  //M1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  if(epsilon>0) M1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);
  return true;
}

