#include "qsp.h"
#include "common.h"
#include "elements/plannerdata_vertex_annotated.h"

#include <ompl/datastructures/PDF.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
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
    G[v].open_neighborhood_distance = d;
  }else{
    double d = checker->Distance(G[v].state);
    G[v].innerApproximationDistance = d;
    G[v].open_neighborhood_distance = d;
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

  // Three cases:
  // state is infeasible => state $q \times X_{k+1} \times X_K$ is infeasible
  // state is sufficient feasible => state $q \times X_{k+1}$ is feasible
  // state is feasible (but not sufficient) => nothing can be said

  Vertex m = CreateNewVertex(state);
  if(isFeasible)
  {
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
  number_of_samples++;
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q_random);
  }else{
    ob::SpaceInformationPtr Q0 = parent->getSpaceInformation();
    base::State *s_X1 = X1->allocState();
    base::State *s_Q0 = Q0->allocState();

    X1_sampler->sampleUniform(s_X1);
    parent->SampleGraph(s_Q0);
    mergeStates(s_Q0, s_X1, q_random);

    X1->freeState(s_X1);
    Q0->freeState(s_Q0);
  }
  return Q1->isValid(q_random);
}

bool QSP::SampleGraph(ob::State *q_random_graph)
{
  //q_random_graph is already allocated! 
  if (pdf_necessary_vertices.empty()){
    std::cout << "ERROR: no necessary vertices." << std::endl;
    std::cout << "vertices : " << boost::num_vertices(G) << std::endl;
    foreach (Vertex v, boost::vertices(G)){
      std::cout << "vertex " << v << " sufficient: " << (G[v].isSufficient?"Yes":"No") << std::endl;
      std::cout << "       " << " necessary: " << ((G[v].isFeasible && !G[v].isSufficient)?"Yes":"No") << std::endl;
    }

    exit(0);
    return false;
  }
  Vertex m = pdf_necessary_vertices.sample(rng_.uniform01());
  si_->copyState(q_random_graph, G[m].state);
  Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, G[m].open_neighborhood_distance);

  return true;
}


void QSP::SetSubGraph( QuotientChart *sibling, uint k )
{
  BaseT::SetSubGraph(sibling, k);
  og::QSP *rhs = dynamic_cast<og::QSP*>(sibling);
  if(rhs == nullptr)
  {
    std::cout << "Invalid operation, setting sub graph from non-qsp class." << std::endl;
    exit(0);
  }
  connectionStrategy_ = KStrategy<Vertex>(3, nn_);

  pdf_necessary_vertices.clear();
  foreach (Vertex v, boost::vertices(G)){
    double d = G[v].open_neighborhood_distance;
    if(!G[v].isSufficient && G[v].isFeasible){
      pdf_necessary_vertices.add(v, d);
      nn_->add(v);
    }
  }
}

ompl::PDF<og::QSP::Vertex> QSP::GetNecessaryVertices() const
{
  return pdf_necessary_vertices;
}

using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;
PlannerDataVertexAnnotated QSP::getPlannerDataVertex(ob::State *state, Vertex v) const
{
  PlannerDataVertexAnnotated p(state);
  if(!G[v].isFeasible){
    p.SetFeasibility(FeasibilityType::INFEASIBLE);
    p.SetInfeasible();
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
