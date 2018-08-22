#include "qsp.h"
#include "common.h"
#include "elements/plannerdata_vertex_annotated.h"

#include <ompl/datastructures/PDF.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QSP::QSP(const ob::SpaceInformationPtr &si, QuotientChart *parent_ ):
  BaseT(si, parent_)
{
  setName("QSP"+std::to_string(id));
  number_of_samples = 0;
  number_of_infeasible_samples = 0;
  number_of_sufficient_samples = 0;

  checker = dynamic_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si->getStateValidityChecker());
  if(!checker){
    checkSufficiency = false;
  }else{
    checkSufficiency = true;
  }

}
void QSP::setup()
{
  BaseT::setup();
  if(setup_){
    Vertex vs = startM_.at(0);
    Vertex vg = goalM_.at(0);
    if(checkSufficiency){
      CheckVertexSufficiency(vs);
      CheckVertexSufficiency(vg);
    }
    G[vs].isFeasible = true;
    G[vg].isFeasible = true;
  }
}

void QSP::CheckVertexSufficiency(Vertex v){
  G[v].isSufficient = checker->IsSufficientFeasible(G[v].state);
  if(G[v].isSufficient){
    number_of_sufficient_samples++;
    double d = checker->SufficientDistance(G[v].state);
    G[v].outerApproximationDistance = d;
  }else{
    double d = checker->Distance(G[v].state);
    G[v].innerApproximationDistance = d;
    pdf_necessary_vertices.add(v, d);
  }
}

QSP::~QSP()
{
}

void QSP::Grow(double t){
  ob::State *state = xstates[0];
  iterations_++;
  bool isFeasible = Sample(state);
  number_of_samples++;

  // Three cases:
  // state is infeasible => state X P_k is infeasible
  // state is sufficient feasible => state X X_{k+1} is feasible
  // state is feasible (but not sufficient) => nothing can be said

  if(isFeasible)
  {
    Vertex m = CreateNewVertex(state);
    G[m].isFeasible = true;
    if(checkSufficiency){
      CheckVertexSufficiency(m);
    }
    ConnectVertexToNeighbors(m);
  }else{
    number_of_infeasible_samples++;
  }
}

bool QSP::Sample(ob::State *q_random)
{
  if(parent == nullptr){
    M1_sampler->sampleUniform(q_random);
  }else{
    ob::SpaceInformationPtr M0 = parent->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    parent->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);
  }
  return M1->isValid(q_random);
}

bool QSP::SampleGraph(ob::State *q_random_graph)
{
  //q_random_graph is already allocated! 
  if (pdf_necessary_vertices.empty()){
    return false;
  }
  Vertex m = pdf_necessary_vertices.sample(rng_.uniform01());
  q_random_graph = G[m].state;

  M1_sampler->sampleUniformNear(q_random_graph, q_random_graph, G[m].innerApproximationDistance);
  return true;
}

using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;
PlannerDataVertexAnnotated QSP::getPlannerDataVertex(ob::State *state, Vertex v) const
{
  PlannerDataVertexAnnotated p(state);
  if(!G[v].isFeasible){
    p.SetFeasibility(FeasibilityType::INFEASIBLE);
  }else{
    if(G[v].isSufficient){
      p.SetFeasibility(FeasibilityType::SUFFICIENT_FEASIBLE);
      p.SetOpenNeighborhoodDistance(G[v].outerApproximationDistance);
    }else{
      p.SetFeasibility(FeasibilityType::FEASIBLE);
      p.SetOpenNeighborhoodDistance(G[v].innerApproximationDistance);
    }
  }
  return p;
}
