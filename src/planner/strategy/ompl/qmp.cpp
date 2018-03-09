#include "qmp.h"
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

QMP::QMP(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  og::PRMBasic(si, previous_)
{
  setName("QMP"+std::to_string(id));
}

QMP::~QMP()
{
}

bool QMP::SampleGraph(ob::State *q_random_graph)
{
  PDF<Edge> pdf = GetEdgePDF();

  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, g_);
  const Vertex v2 = boost::target(e, g_);
  const ob::State *from = stateProperty_[v1];
  const ob::State *to = stateProperty_[v2];

  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
  //M1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  //M1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);

  return true;
}

ompl::PDF<og::PRMBasic::Edge> QMP::GetEdgePDF()
{
  PDF<Edge> pdf;
  double t = rng_.uniform01();
  if(t<percentageSamplesOnShortestPath)
  {
    //shortest path heuristic
    foreach (Edge e, boost::edges(g_))
    {
      const Vertex v1 = boost::source(e, g_);
      const Vertex v2 = boost::target(e, g_);
      if(onShortestPath_[v1] && onShortestPath_[v2]){
        ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
        pdf.add(e, weight.value());
      }
    }

  }else{
    foreach (Edge e, boost::edges(g_))
    {
      const Vertex v1 = boost::source(e, g_);

      if(sameComponent(v1, startM_.at(0))){
        //const std::vector<Vertex> &neighbors = connectionStrategy_(v1);
        //pdf.add(e, 1.0/(100*neighbors.size()));
        ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
        pdf.add(e, weight.value());
      }
    }
  }
  return pdf;
}
