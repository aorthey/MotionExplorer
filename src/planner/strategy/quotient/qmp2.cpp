#include "qmp2.h"
#include "planner/validitychecker/validity_checker_ompl.h"

#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QMP2::QMP2(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  og::QuotientGraph(si, previous_)
{
  setName("QMP2"+std::to_string(id));
}

QMP2::~QMP2()
{
  samplesOnShortestPath = 0;
}

bool QMP2::SampleGraph(ob::State *q_random_graph)
{
  PDF<Edge> pdf = GetEdgePDF();

  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, G);
  const Vertex v2 = boost::target(e, G);
  const ob::State *from = G[v1].state;
  const ob::State *to = G[v2].state;

  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
  //M1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  M1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);
  return true;
}

ompl::PDF<og::QuotientGraph::Edge> QMP2::GetEdgePDF()
{
  PDF<Edge> pdf;
  double t = rng_.uniform01();
  if(t<percentageSamplesOnShortestPath)
  {
    //shortest path heuristic
    foreach (Edge e, boost::edges(G))
    {
      const Vertex v1 = boost::source(e, G);
      const Vertex v2 = boost::target(e, G);
      if(G[v1].on_shortest_path && G[v2].on_shortest_path){
        ob::Cost weight = G[e].getCost();
        pdf.add(e, weight.value());
      }
    }

  }else{
    foreach (Edge e, boost::edges(G))
    {
      const Vertex v1 = boost::source(e, G);

      if(sameComponent(v1, startM_.at(0))){
        ob::Cost weight = G[e].getCost();
        pdf.add(e, weight.value());
      }
    }
  }
  return pdf;
}
