#include "q_rrt_connect.h"
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

QRRTConnect::QRRTConnect(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
  setName("QRRTConnect"+std::to_string(id));
}


void QRRTConnect::clear()
{
  vpdf.clear();
  q_last_added = nullptr;
}
void QRRTConnect::Grow(double t)
{
  if(parent == nullptr) BaseT::Grow(t);
  else{
    uint Nvertices_before = GetNumberOfVertices();
    BaseT::Grow(t);
    uint Nvertices_after = GetNumberOfVertices();

    ////Update PDF of sampled point on qspace
    bool success_extension = (Nvertices_after > Nvertices_before);

    QRRTConnect *quotient = dynamic_cast<QRRTConnect*>(parent);

    Configuration *q_quotient = quotient->q_last_added;
    if(q_quotient != nullptr){
      //PrintConfiguration(q_quotient);
      const Vertex q_idx = q_quotient->index;
      Graph Gp = quotient->GetGraph();
      uint &T = Gp[q_idx]->total_connection_attempts;
      uint &Ts = Gp[q_idx]->successful_connection_attempts;

      T++;
      if(success_extension) Ts++;

      //update pdf
      double v_value = 1.0 / ((double)10*Ts+1.0);
      quotient->vpdf.update(static_cast<PDF_Element*>(q_quotient->GetPDFElement()), v_value);
      q_quotient = nullptr;
    }

  }
}

bool QRRTConnect::Sample(ob::State *q_random)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q_random);
  }else{
    if(X1_dimension>0)
    {
      X1_sampler->sampleUniform(s_X1_tmp);
      parent->SampleQuotient(s_Q0_tmp);
      MergeStates(s_Q0_tmp, s_X1_tmp, q_random);
    }else{
      parent->SampleQuotient(q_random);
    }
  }
  return true;
}

bool QRRTConnect::SampleQuotient(ob::State *q_random_graph)
{
  q_last_added = vpdf.sample(rng_.uniform01());
  Q1->getStateSpace()->copyState(q_random_graph, q_last_added->state);
  // const Vertex v = boost::random_vertex(G, rng_boost);
  // Q1->getStateSpace()->copyState(q_random_graph, G[v]->state);
  return true;
}

QuotientGraph::Vertex QRRTConnect::AddConfiguration(Configuration *q)
{
  Vertex m = BaseT::AddConfiguration(q);
  PDF_Element *q_element = vpdf.add(G[m], 1);
  G[m]->SetPDFElement(q_element);
  //q_last_added = q;
  return m;
}
