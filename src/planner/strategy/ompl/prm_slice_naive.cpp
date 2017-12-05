#include "prm_slice_naive.h"
#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH
//
//  Visualization of CSpace Decomposition
//
// [    ][    ]
// [    ][    ]
// [    ][ M0 ]
// [ M1 ][____]
// [    ][    ]
// [    ][ C1 ]
// [    ][    ]
//
// whereby 
// M1 = M1
// M0 = previous->getspaceinformation()
// C1 = C1
//
// Standard PRM is sampling in M1
// PRMSlice is sampling in G0 x C0, whereby G0 is the roadmap on M0
//
//
//Multilevel M0 \subspace M1 \subspace M2
//
// [    ][    ][    ]
// [    ][    ][ M0 ]
// [    ][    ][    ]
// [    ][ M1 ][____]
// [    ][    ]
// [ M2 ][    ]
// [    ][    ]
// [    ][____]
// [    ]
// [    ]
// [____]
//
// [    ][    ][    ]
// [    ][    ][ C0 ]
// [    ][    ][    ]
// [    ][ M1 ][____]
// [    ][    ][    ]
// [ M2 ][    ][ C1 ]
// [    ][    ][    ]
// [    ][____][____]
// [    ][    ][    ]
// [    ][ C2 ][ C2 ]
// [____][____][____]

PRMSliceNaive::PRMSliceNaive(const ob::SpaceInformationPtr &si, PRMSliceNaive *previous_ ):
  og::PRMSlice(si), previous(previous_), M1(si)
{
  const StateSpacePtr M1_space = M1->getStateSpace();
  M0_subspaces = 0;
  M1_subspaces = 0;

  if(previous == nullptr){
    std::cout << "Level 0 SliceSpace" << std::endl;
    //C1 = std::make_shared<SpaceInformation>(M1);
  }else{
    std::cout << "Level 1 SliceSpace" << std::endl;
    const StateSpacePtr M0_space = previous->getSpaceInformation()->getStateSpace();

    //create standalone CSpace M1/M0
    if(!M1_space->isCompound()){
      std::cout << "Cannot Split a non-compound configuration space" << std::endl;
      exit(0);
    }else{

      //get count of subspaces in M0
      if(!M0_space->isCompound()){
        M0_subspaces=1;
      }else{
        M0_subspaces = M0_space->as<ob::CompoundStateSpace>()->getSubspaceCount();
      }

      //get count of subspaces in M1
      if(!M1_space->isCompound()){
        std::cout << "M1 needs to be compound state" << std::endl;
        exit(0);
      }
      ob::CompoundStateSpace *M1_compound = M1_space->as<ob::CompoundStateSpace>();
      const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
      M1_subspaces = M1_decomposed.size();

      std::cout << "M0 contains " << M0_subspaces << " subspaces." << std::endl;
      std::cout << "M1 contains " << M1_subspaces << " subspaces." << std::endl;

      //sanity check it
      if(M0_subspaces >= M1_subspaces){
        std::cout << "Cannot reduce to a bigger cspace. We need M0 < M1." << std::endl;
        exit(0);
      }

      //get C1
      C1_subspaces = M1_subspaces - M0_subspaces;

      StateSpacePtr C1_space;
      if(C1_subspaces<=1){
        C1_space = M1_decomposed.at(M0_subspaces);
      }else{
        for(uint k = M0_subspaces; k < M0_subspaces+C1_subspaces; k++){
          C1_space->as<ob::CompoundStateSpace>()->addSubspace(M1_decomposed.at(k),1);
        }
      }
      C1 = std::make_shared<SpaceInformation>(C1_space);

      //get count of subspaces in C1
      uint C1_subspaces = 0;
      if(!C1_space->isCompound()){
        C1_subspaces=1;
      }else{
        C1_subspaces = C1_space->as<ob::CompoundStateSpace>()->getSubspaceCount();
      }
      std::cout << "C1 contains " << C1_subspaces << " subspaces." << std::endl;
    }
    if (!C1_sampler){
      C1_sampler = C1->allocStateSampler();
    }

  }
  //const StateValidityCheckerPtr checker = si->getStateValidityChecker();
  //C1->setStateValidityChecker(checker);

}

PRMSliceNaive::~PRMSliceNaive(){
}

double PRMSliceNaive::getSamplingDensity(){
  if(previous == nullptr){
    return (double)num_vertices(g_)/(double)M1->getSpaceMeasure();
  }else{
    //get graph length
    double Lprev = 0.0;
    const Graph gprev = previous->getRoadmap();
    foreach (Edge e, boost::edges(gprev))
    {
      EdgeProperty ep = get(boost::edge_weight_t(), gprev, e);
      ob::Cost weight = ep.getCost();
      Lprev += weight.value();
    }
    return (double)num_vertices(g_)/(C1->getSpaceMeasure()+Lprev);
  }
}


void PRMSliceNaive::getPlannerData(base::PlannerData &data) const{
  PRMSlice::getPlannerData(data);
  std::cout << "Number of start states: " << startM_.size() << std::endl;
  std::cout << "Number of goal  states: " << goalM_.size() << std::endl;
}

bool PRMSliceNaive::Sample(ob::State *workState){
  if(previous == nullptr){
    //without any underlying CSpace, the behavior is equal to PRM
    return sampler_->sample(workState);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1

    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, workState);

    return M1->isValid(workState);
  }
}

bool PRMSliceNaive::SampleGraph(ob::State *workState){
  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
    pdf.add(e, weight.value());
  }
  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, g_);
  const Vertex v2 = boost::target(e, g_);
  const ob::State *from = stateProperty_[v1];
  const ob::State *to = stateProperty_[v2];

  M1->getStateSpace()->interpolate(from, to, t, workState);

  if(t<0.5) lastVertexSampled = v1;
  else lastVertexSampled = v2;

  lastTSampled = t;
  isSampled = true;

  return true;

}

void PRMSliceNaive::ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const{
  State **q_comps = q->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(C1_subspaces > 1){
    State **sC1_comps = qC1->as<CompoundState>()->components;
    for(uint k = M0_subspaces; k < subspaces.size(); k++){
      subspaces[k]->copyState( sC1_comps[k-M0_subspaces], q_comps[k]);
    }
  }else{
    subspaces[M0_subspaces]->copyState( qC1, q_comps[M0_subspaces]);
  }
}

void PRMSliceNaive::ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const{
  State **q_comps = q->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(M0_subspaces > 1){
    State **sM0_comps = qM0->as<CompoundState>()->components;
    for(uint k = 0; k < M0_subspaces; k++){
      subspaces[k]->copyState( sM0_comps[k], q_comps[k]);
    }
  }else{
    subspaces[0]->copyState( qM0, q_comps[0]);
  }
}
void PRMSliceNaive::mergeStates(ob::State *qM0, ob::State *qC1, ob::State *qM1){
  //
  //input : qM0 \in M0, qC1 \in C1
  //output: qM1 = qM0 \circ qC1 \in M1
  State **qM1_comps = qM1->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(M0_subspaces > 1){
    State **sM0_comps = qM0->as<CompoundState>()->components;
    for(uint k = 0; k < M0_subspaces; k++){
      subspaces[k]->copyState( qM1_comps[k], sM0_comps[k]);
    }
  }else{
    subspaces[0]->copyState( qM1_comps[0], qM0);
  }
  if(C1_subspaces > 1){
    State **sC1_comps = qC1->as<CompoundState>()->components;
    for(uint k = M0_subspaces; k < subspaces.size(); k++){
      subspaces[k]->copyState( qM1_comps[k], sC1_comps[k-M0_subspaces]);
    }
  }else{
    subspaces[M0_subspaces]->copyState( qM1_comps[M0_subspaces], qC1);
  }
}

//og::PRMPlain::Vertex PRMSliceNaive::addMilestone(base::State *state)
//{
//  return PRMPlain::addMilestone(state);
//
//  //Vertex m = boost::add_vertex(g_);
//  //stateProperty_[m] = state;
//  //totalConnectionAttemptsProperty_[m] = 1;
//  //successfulConnectionAttemptsProperty_[m] = 0;
//  //if(previous != nullptr && previous->isSampled){
//  //  std::cout << lastVertexSampled << std::endl;
//  //  associatedVertexProperty_[m] = lastVertexSampled;
//  //}
//
//  //disjointSets_.make_set(m);
//  //const std::vector<Vertex> &neighbors = connectionStrategy_(m);
//
//  //foreach (Vertex n, neighbors)
//  //{
//  //  if (connectionFilter_(n, m))
//  //  {
//  //    totalConnectionAttemptsProperty_[m]++;
//  //    totalConnectionAttemptsProperty_[n]++;
//  //    if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
//  //    {
//  //      successfulConnectionAttemptsProperty_[m]++;
//  //      successfulConnectionAttemptsProperty_[n]++;
//  //      EdgeProperty properties(opt_->motionCost(stateProperty_[n], stateProperty_[m]));
//  //      boost::add_edge(n, m, properties, g_);
//  //      uniteComponents(n, m);
//  //    }
//  //  }
//  //}
//
//  //nn_->add(m);
//
//  //return m;
//}

void PRMSliceNaive::setup(){
  og::PRMBasic::setup();
  nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return distanceFunction(a,b);
                           });

}

double PRMSliceNaive::distanceFunction(const Vertex a, const Vertex b) const
{
  if(previous == nullptr){
    return si_->distance(stateProperty_[a], stateProperty_[b]);
  }else{
    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();

    ob::State* qa = stateProperty_[a];
    ob::State* qb = stateProperty_[b];

    ob::State* qaC1 = C1->allocState();
    ob::State* qbC1 = C1->allocState();
    ob::State* qaM0 = M0->allocState();
    ob::State* qbM0 = M0->allocState();

    ExtractC1Subspace(qa, qaC1);
    ExtractC1Subspace(qb, qbC1);
    ExtractM0Subspace(qa, qaM0);
    ExtractM0Subspace(qb, qbM0);

    const Vertex saM0 = associatedVertexSourceProperty_[a];
    const Vertex sbM0 = associatedVertexSourceProperty_[b];
    const Vertex taM0 = associatedVertexTargetProperty_[a];
    const Vertex tbM0 = associatedVertexTargetProperty_[b];
    std::cout << saM0 << " <-> " << sbM0 << std::endl;

    double d0 = previous->distanceGraphFunction(qaM0, qbM0, saM0, sbM0);
    double d1 = C1->distance(qaC1, qbC1);

    C1->freeState(qaC1);
    C1->freeState(qbC1);
    M0->freeState(qaM0);
    M0->freeState(qbM0);

    return d0 + d1;
    //return si_->distance(stateProperty_[a], stateProperty_[b]);
  }
}

double PRMSliceNaive::distanceGraphFunction(ob::State *qa, ob::State *qb, const Vertex va, const Vertex vb)
{
  //@TODO: compute distance qa to va, and subtract or add to total cost
  //search on graph between vaM0 and vbM0

  ob::PathPtr sol = constructSolution(va, vb);
  if(sol==nullptr){
    return +dInf;
  }else{
    return sol->length();
  }
}

ob::PlannerStatus PRMSliceNaive::Init()
{
  PRMSlice::Init();
  if(previous!=nullptr){
    for(uint k = 0; k < startM_.size(); k++){
      associatedVertexSourceProperty_[startM_.at(k)] = previous->startM_.at(k);
      associatedVertexTargetProperty_[startM_.at(k)] = previous->startM_.at(k);
    }
    for(uint k = 0; k < goalM_.size(); k++){
      associatedVertexSourceProperty_[goalM_.at(k)] = previous->goalM_.at(k);
      associatedVertexTargetProperty_[goalM_.at(k)] = previous->startM_.at(k);
    }
  }
}
