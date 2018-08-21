#include "qsp.h"
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
      G[vs].isSufficient = checker->IsSufficientFeasible(G[vs].state);
      G[vg].isSufficient = checker->IsSufficientFeasible(G[vg].state);
    }
    G[vs].isFeasible = true;
    G[vg].isFeasible = true;
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

  Vertex m = CreateNewVertex(state);
  if(isFeasible)
  {
    G[m].isFeasible = true;
    if(checkSufficiency){
      bool sufficient = checker->IsSufficientFeasible(state);
      if(sufficient){
        number_of_sufficient_samples++;
        double d = checker->SufficientDistance(state);
        G[m].isSufficient = true;
        G[m].outerApproximationDistance = d;
      }else{
        double d = checker->Distance(state);
        G[m].isSufficient = false;
        G[m].innerApproximationDistance = d;
        pdf_necessary_vertices.add(m, d);
      }
    }
    addMilestone(state);
  }else{
    number_of_infeasible_samples++;
  }
}

bool QSP::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(parent == nullptr){
    return M1_valid_sampler->sample(q_random);
  }else{
    ob::SpaceInformationPtr M0 = parent->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    parent->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);

    return M1->isValid(q_random);
  }
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
void QSP::getPlannerData(ob::PlannerData &data) const
{

  //###########################################################################
  //Get Path for this chart
  //###########################################################################
  std::vector<int> path;
  path.push_back(GetHorizontalIndex());
  og::QuotientChart *parent = static_cast<og::QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    path.push_back(parent->GetHorizontalIndex());
    parent = static_cast<og::QuotientChart*>(parent->GetParent());
  }

  std::reverse(std::begin(path), std::end(path));
  
  //###########################################################################
  //Get Data from this chart
  //###########################################################################
  std::cout << "vertices: " << boost::num_vertices(G) << std::endl;
  foreach (const Vertex v, boost::vertices(G))
  {
    PlannerDataVertexAnnotated p(G[v].state);

    p.SetLevel(level);
    p.SetPath(path);

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
      std::cout << "volume: " << p.GetOpenNeighborhoodDistance() << std::endl;
    }

    if(G[v].start) data.addStartVertex(p);
    else if(G[v].goal) data.addGoalVertex(p);
    else data.addVertex(p);
  }

  foreach (const Edge e, boost::edges(G))
  {
    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);

    PlannerDataVertexAnnotated *p1 = dynamic_cast<PlannerDataVertexAnnotated*>(&data.getVertex(v1));
    PlannerDataVertexAnnotated *p2 = dynamic_cast<PlannerDataVertexAnnotated*>(&data.getVertex(v2));
    data.addEdge(*p1,*p2);

  }
  std::cout << "Samples: " << number_of_samples << " (" 
    << 100.0*(double)number_of_infeasible_samples/(double)number_of_samples << "\% infeasible | "
    << 100.0*(double)(number_of_samples-number_of_infeasible_samples)/(double)number_of_samples << "\% feasible | "
    << 100.0*(double)number_of_sufficient_samples/(double)number_of_samples << "\% sufficient)"
    << std::endl;

  //###########################################################################
  //recursively traverse other charts
  //###########################################################################
  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);

}
