#include "qmp.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"

#include <ompl/datastructures/PDF.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp> 

#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QMP::QMP(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  og::QuotientGraph(si, previous_)
{
  setName("QMP"+std::to_string(id));
}

QMP::~QMP()
{
  samplesOnShortestPath = 0;
}

typedef boost::minstd_rand RNGType;
RNGType rng;

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
    percentageSamplesOnShortestPath = exp(-pow(((double)samplesOnShortestPath++/1000.0),2));

    for(uint k = 0; k < shortestVertexPath_.size()-1; k++){
      Vertex v1 = shortestVertexPath_.at(k);
      Vertex v2 = shortestVertexPath_.at(k+1);
      Edge e = boost::edge(v1,v2,G).first;
      pdf.add(e, G[e].getCost().value());
    }
    e = pdf.sample(rng_.uniform01());
  }else{
    e = boost::random_edge(G, rng);
    while(!sameComponent(boost::source(e, G), startM_.at(0)))
    {
      e = boost::random_edge(G, rng);
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

ompl::PDF<og::QuotientGraph::Edge> QMP::GetEdgePDF()
{
  PDF<Edge> pdf;
  double t = rng_.uniform01();
  if(t<percentageSamplesOnShortestPath)
  {
    //shortest path heuristic
    percentageSamplesOnShortestPath = exp(-pow(((double)samplesOnShortestPath++/10000.0),2));

    for(uint k = 0; k < shortestVertexPath_.size()-1; k++){
      Vertex v1 = shortestVertexPath_.at(k);
      Vertex v2 = shortestVertexPath_.at(k+1);
      Edge e = boost::edge(v1,v2,G).first;
      pdf.add(e, G[e].getCost().value());
    }

  }else{
    //Random Edge (RE) sampling (suffers from high sampling concentrations at
    //vertices with many incoming edges, not ideal, but fast)
    //foreach (Edge e, boost::edges(G))
    //{
    //  const Vertex v1 = boost::source(e, G);

    //  if(sameComponent(v1, startM_.at(0))){
    //    ob::Cost weight = get(boost::edge_weight_t(), Ge).getCost();
    //    pdf.add(e, weight.value());
    //  }
    //}
    ////Random Node Edge (RNE) sampling (better, but still no guarantee of
    //uniformity. Might be good to investigate random walk samplers)
    foreach (Vertex v, boost::vertices(G))
    {
      if(sameComponent(v, startM_.at(0))){
        vpdf.add(v,1);
      }
    }
    Vertex v = vpdf.sample(rng_.uniform01());
    std::pair<IEIterator, IEIterator> iterators = boost::in_edges(boost::vertex(v, G), G);
    for (IEIterator iter = iterators.first; iter != iterators.second; ++iter)
    {
      ob::Cost weight = G[*iter].getCost();
      pdf.add(*iter, weight.value());
    }
  }
  return pdf;
}
