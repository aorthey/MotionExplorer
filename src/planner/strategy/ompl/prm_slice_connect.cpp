#include "prm_slice_connect.h"
#include "planner/cspace/cspace.h"

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
#define DEBUG 0
namespace ompl
{
  namespace magic
  {
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
    static const double ROADMAP_BUILD_TIME = 0.01;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}
uint PRMSliceConnect::counter = 0;
void PRMSliceConnect::resetCounter(){
  PRMSliceConnect::counter = 0;
}

PRMSliceConnect::PRMSliceConnect(const ob::SpaceInformationPtr &si, PRMSliceConnect *previous_ ):
  og::PRMBasic(si), previous(previous_), M1(si)
{

  const StateSpacePtr M1_space = M1->getStateSpace();
  M0_subspaces = 0;
  M1_subspaces = 0;

  id = counter++;
  std::cout << "--- Level " << id << " SliceSpace" << std::endl;
  if(previous == nullptr){
  }else{
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


      //sanity check it
      if(M0_space->getDimension() >= M1_space->getDimension()){
        std::cout << "Cannot reduce to a bigger cspace. We need M0 < M1." << std::endl;
        std::cout << "M0 dimension: " << M0_space->getDimension() << std::endl;
        std::cout << "M1 dimension: " << M1_space->getDimension() << std::endl;
        exit(0);
      }

      if(M0_subspaces < M1_subspaces){
        //get C1
        C1_subspaces = M1_subspaces - M0_subspaces;

        if(C1_subspaces<=1){
          StateSpacePtr C1_space = M1_decomposed.at(M0_subspaces);
          C1 = std::make_shared<SpaceInformation>(C1_space);
        }else{
          StateSpacePtr C1_space = std::make_shared<ompl::base::CompoundStateSpace>();
          for(uint k = M0_subspaces; k < M0_subspaces+C1_subspaces; k++){
            C1_space->as<ob::CompoundStateSpace>()->addSubspace(M1_decomposed.at(k),1);
          }
          C1 = std::make_shared<SpaceInformation>(C1_space);
        }
      }else{
        StateSpacePtr M1_back = M1_decomposed.back();
        if((M1_back->getType() == base::STATE_SPACE_SO3) && (M0_space->getDimension() == 5)){
          StateSpacePtr C1_space = (std::make_shared<ob::SO2StateSpace>());
          C1 = std::make_shared<SpaceInformation>(C1_space);
          C1_subspaces = 1;
        }else{
          std::cout << "Subspaces do not match. We can only handle this in the case of S1xS1 subset SO(3)" << std::endl;
          exit(0);
        }

      }

    }
    std::cout << "M0 (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
    std::cout << "M1 (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
    std::cout << "C1 (dimension: " << C1->getStateDimension() << ") contains " << C1_subspaces  << " subspaces of type " << C1->getStateSpace()->getName() << " " << C1->getStateSpace()->getType() << std::endl;
    if (!C1_sampler){
      C1_sampler = C1->allocStateSampler();
    }

  }
  //const StateValidityCheckerPtr checker = si->getStateValidityChecker();
  //C1->setStateValidityChecker(checker);

}

PRMSliceConnect::~PRMSliceConnect(){
}


ob::PlannerStatus PRMSliceConnect::Init()
{
  checkValidity();
  auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

  if (goal == nullptr){
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    exit(0);
  }

  while (const ob::State *st = pis_.nextStart()){
    startM_.push_back(addMilestone(si_->cloneState(st)));
  }
  if (startM_.empty()){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }
  if (!goal->couldSample()){
    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
    exit(0);
  }

  if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()){
    const ob::State *st = pis_.nextGoal();
    if (st != nullptr){
      goalM_.push_back(addMilestone(si_->cloneState(st)));
    }
  }
  unsigned long int nrStartStates = boost::num_vertices(g_);
  OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

  if(previous!=nullptr){
    for(uint k = 0; k < startM_.size(); k++){
      associatedVertexSourceProperty_[startM_.at(k)] = previous->startM_.at(k);
      associatedVertexTargetProperty_[startM_.at(k)] = previous->startM_.at(k);
    }
    for(uint k = 0; k < goalM_.size(); k++){
      associatedVertexSourceProperty_[goalM_.at(k)] = previous->goalM_.at(k);
      associatedVertexTargetProperty_[goalM_.at(k)] = previous->goalM_.at(k);
    }
  }
}

void PRMSliceConnect::ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const{
  ob::State **q_comps = q->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(C1_subspaces > 1){
    ob::State **sC1_comps = qC1->as<CompoundState>()->components;
    for(uint k = M0_subspaces; k < subspaces.size(); k++){
      subspaces[k]->copyState( sC1_comps[k-M0_subspaces], q_comps[k]);
    }
  }else{

    StateSpacePtr M1_back = subspaces.back();
    if((M1_back->getType() == base::STATE_SPACE_SO3) && (C1->getStateSpace()->getDimension() == 1)){

      ob::SO3StateSpace::StateType *SO3 = &q->as<ob::SE3StateSpace::StateType>()->rotation();

      std::vector<double> EulerXYZ = CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace( SO3);

      qC1->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(2);

    }else{
      subspaces[M0_subspaces]->copyState( qC1, q_comps[M0_subspaces]);
    }
  }
}

void PRMSliceConnect::ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const{
  ob::State **q_comps = q->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(M0_subspaces > 1){
    ob::State **sM0_comps = qM0->as<CompoundState>()->components;

    StateSpacePtr M1_back = subspaces.back();
    if((M1_back->getType() == base::STATE_SPACE_SO3) && (C1->getStateSpace()->getDimension() == 1)){


      ob::SO3StateSpace::StateType *SO3 = &q->as<ob::SE3StateSpace::StateType>()->rotation();

      std::vector<double> EulerXYZ = CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace( SO3);

      //qC1->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(2);

      subspaces[0]->copyState( sM0_comps[0], q_comps[0]);
      sM0_comps[1]->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(0);
      sM0_comps[2]->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(1);

      //std::cout << "special case" << std::endl;

    }else{
      for(uint k = 0; k < M0_subspaces; k++){
        subspaces[k]->copyState( sM0_comps[k], q_comps[k]);
      }
    }
  }else{
    subspaces[0]->copyState( qM0, q_comps[0]);
  }
}

void PRMSliceConnect::mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1){
  //input : qM0 \in M0, qC1 \in C1
  //output: qM1 = qM0 \circ qC1 \in M1
  ob::State **qM1_comps = qM1->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  StateSpacePtr M1_back = subspaces.back();
  if((M1_back->getType() == base::STATE_SPACE_SO3) && (C1->getStateSpace()->getDimension() == 1)){
    //special case

    ob::State **sM0_comps = qM0->as<CompoundState>()->components;
    subspaces[0]->copyState( qM1_comps[0], sM0_comps[0]);

    double r = sM0_comps[1]->as<ob::SO2StateSpace::StateType>()->value;
    double p = sM0_comps[2]->as<ob::SO2StateSpace::StateType>()->value;
    double y = qC1->as<ob::SO2StateSpace::StateType>()->value;

    ob::SE3StateSpace::StateType *qM1SE3 = qM1->as<ob::SE3StateSpace::StateType>();
    ob::SO3StateSpace::StateType *qM1SO3 = &qM1SE3->rotation();

    CSpaceOMPL::OMPLSO3StateSpaceFromEulerXYZ(r, p, y, qM1_comps[1]->as<ob::SO3StateSpace::StateType>() );

  }else{
    if(M0_subspaces > 1){
      ob::State **sM0_comps = qM0->as<CompoundState>()->components;
      for(uint k = 0; k < M0_subspaces; k++){
        subspaces[k]->copyState( qM1_comps[k], sM0_comps[k]);
      }
    }else{
      subspaces[0]->copyState( qM1_comps[0], qM0);
    }
    if(C1_subspaces > 1){
      ob::State **sC1_comps = qC1->as<CompoundState>()->components;
      for(uint k = M0_subspaces; k < subspaces.size(); k++){
        subspaces[k]->copyState( qM1_comps[k], sC1_comps[k-M0_subspaces]);
      }
    }else{
      subspaces[M0_subspaces]->copyState( qM1_comps[M0_subspaces], qC1);
    }
  }
}

og::PRMBasic::Vertex PRMSliceConnect::addMilestone(base::State *state)
{
  Vertex m = PRMBasic::addMilestone(state);
    
  if(previous != nullptr && previous->isSampled){
    associatedVertexSourceProperty_[m] = previous->lastSourceVertexSampled;
    associatedVertexTargetProperty_[m] = previous->lastTargetVertexSampled;
    associatedTProperty_[m] = previous->lastTSampled;
  }

  return m;
}

void PRMSliceConnect::setup(){
  og::PRMBasic::setup();
  nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return Distance(a,b);
                           });

}


double PRMSliceConnect::getSamplingDensity(){
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


void PRMSliceConnect::getPlannerData(base::PlannerData &data) const{
  PRMBasic::getPlannerData(data);
  std::cout << "Number of start states: " << startM_.size() << std::endl;
  std::cout << "Number of goal  states: " << goalM_.size() << std::endl;
}

bool PRMSliceConnect::Sample(ob::State *workState){
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

bool PRMSliceConnect::SampleGraph(ob::State *workState){
  PDF<Edge> pdf;
  Vertex v1o,v2o;
  foreach (Edge e, boost::edges(g_))
  {
    ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
    pdf.add(e, weight.value());
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);

    if(DEBUG){
      if(v1o==v1 && v2o==v2){
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Double edge (num edges: " << num_edges(g_) << ")" << std::endl;
        std::cout << "Edge: (" << v1 << "," << v2 << ") weight: "<<weight.value() << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        uint ctr = 0;
        foreach (Edge e, boost::edges(g_))
        {
          ctr++;
        }
        std::cout << "edges counted: " << ctr << std::endl;
        exit(0);
      }
      if(v1o==v2 && v2o==v1){
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Double edge (num edges: " << num_edges(g_) << ")" << std::endl;
        std::cout << "Edge: (" << v1 << "," << v2 << ") weight: "<<weight.value() << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        uint ctr = 0;
        foreach (Edge e, boost::edges(g_))
        {
          ctr++;
        }
        std::cout << "edges counted: " << ctr << std::endl;
        exit(0);
      }

      v1o = v1;
      v2o = v2;
    }
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

  if(DEBUG){
    std::cout << "SampleGraph" << std::endl;
    std::cout << v1 << std::endl;
    std::cout << v2 << std::endl;
    si_->printState(from);
    si_->printState(to);
    M1->printState(workState);
  }

  lastSourceVertexSampled = v1;
  lastTargetVertexSampled = v2;
  lastTSampled = t;

  isSampled = true;

  return true;

}

double PRMSliceConnect::Distance(const Vertex a, const Vertex b) const
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

    const Vertex vsaM0 = associatedVertexSourceProperty_[a];
    const Vertex vsbM0 = associatedVertexSourceProperty_[b];
    const Vertex vtaM0 = associatedVertexTargetProperty_[a];
    const Vertex vtbM0 = associatedVertexTargetProperty_[b];

    double ta = associatedTProperty_[a];
    double tb = associatedTProperty_[b];

    ob::PathPtr sol = previous->GetShortestPathOffsetVertices( qaM0, qbM0, vsaM0, vsbM0, vtaM0, vtbM0);
    double d0 = +dInf;
    if(sol!=nullptr){
      d0 = sol->length();
    }
    if(DEBUG) std::cout << "path distance: " << d0 << std::endl;

    double d1 = C1->distance(qaC1, qbC1);

    C1->freeState(qaC1);
    C1->freeState(qbC1);
    M0->freeState(qaM0);
    M0->freeState(qbM0);

    return d0 + d1;
    //return si_->distance(stateProperty_[a], stateProperty_[b]);
  }
}

ob::PathPtr PRMSliceConnect::GetShortestPathOffsetVertices( const ob::State *qa, const ob::State *qb, 
  const Vertex vsa, const Vertex vsb, const Vertex vta, const Vertex vtb)
{
  //###########################################################################
  //construct modified graph
  //###########################################################################
  Vertex va;
  Vertex vb;

  if(vsa!=vta && vsb!=vtb){
    const Edge ea = boost::edge(vsa,vta,g_).first;
    const Edge eb = boost::edge(vsb,vtb,g_).first;
    if(ea==eb){
      ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
      return path;
    }
  }
  if( (vsa==vta && vsa==vsb) ||
      (vsa==vta && vsa==vtb) ||
      (vsb==vtb && vsb==vsa) ||
      (vsb==vtb && vsb==vta)){
    ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
    return path;
  }
  if(DEBUG) std::cout << std::string(80, '-') << std::endl;
  if(DEBUG) std::cout << "graph (edges: " << num_edges(g_) << ")" << std::endl;

  bool isAonEdge = true;
  bool isBonEdge = true;
  if(vsa==vta) isAonEdge = false;
  if(vsb==vtb) isBonEdge = false;

  ob::Cost dsa,dta,dsb,dtb;

  if(isAonEdge){
    if(!boost::edge(vsa,vta,g_).second){
      std::cout << "edge vsa-vta does not exist" << std::endl;
      exit(0);
    }
    boost::remove_edge(vsa, vta, g_);
    va = add_vertex(g_);
    stateProperty_[va] = si_->cloneState(qa);
    if(DEBUG) std::cout << "rm edge "<< vsa << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;

    dsa = opt_->motionCost(stateProperty_[vsa], stateProperty_[va]);
    dta = opt_->motionCost(stateProperty_[va], stateProperty_[vta]);
    std::pair<Edge,bool> e1 = boost::add_edge(vsa, va, EdgeProperty(dsa), g_);
    std::pair<Edge,bool> e2 = boost::add_edge(va, vta, EdgeProperty(dta), g_);
    if(!e1.second){
      std::cout << "edge 1 does not exists" << std::endl;
      exit(0);
    }
    if(!e2.second){
      std::cout << "edge 2 does not exists" << std::endl;
      exit(0);
    }

    uniteComponents(vsa, va);
    uniteComponents(va, vta);

    if(DEBUG) std::cout << "added edge vsa-va-vta " << vsa << "," << va << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;
  }else{
    va = vsa;
    //stateProperty_[va] = si_->cloneState(qa);
  }

  if(isBonEdge){
    if(!boost::edge(vsb,vtb,g_).second){
      std::cout << "edge vsa-vta does not exist" << std::endl;
      exit(0);
    }
    boost::remove_edge(vsb, vtb, g_);
    vb = add_vertex(g_);
    stateProperty_[vb] = si_->cloneState(qb);
    if(DEBUG) std::cout << "rm edge "<< vsb << "," << vtb << " (edges: " << num_edges(g_) << ")" << std::endl;

    dsb = opt_->motionCost(stateProperty_[vsb], stateProperty_[vb]);
    dtb = opt_->motionCost(stateProperty_[vb], stateProperty_[vtb]);
    boost::add_edge(vsb, vb, EdgeProperty(dsb), g_);
    boost::add_edge(vb, vtb, EdgeProperty(dtb), g_);
    //if(!e1.second){
    //  std::cout << "edge 1 does not exists" << std::endl;
    //  exit(0);
    //}
    //if(!e2.second){
    //  std::cout << "edge 2 does not exists" << std::endl;
    //  exit(0);
    //}
    uniteComponents(vsb, vb);
    uniteComponents(vb, vtb);

    if(DEBUG) std::cout << "added edge vsb-vb-vtb " << vsb << "," << vb << "," << vtb << " (edges: " << num_edges(g_) << ")" << std::endl;
  }else{
    vb = vsb;
    //stateProperty_[vb] = qb;
  }


  //###########################################################################
  //search in modified graph
  //###########################################################################

  bool same_component = sameComponent(va, vb);
  ob::PathPtr sol = nullptr;
  if(same_component){
    sol = constructSolution(va, vb);
  }else{
    std::cout << "not same component" << std::endl;
    exit(0);
  }

  //###########################################################################
  //return to former graph
  //###########################################################################

  if(isBonEdge){
    if(!boost::edge(vsb,vb,g_).second){
      std::cout << "edge " << vsb << "," << vb << " does not exist" << std::endl;
      exit(0);
    }
    if(!boost::edge(vb,vtb,g_).second){
      std::cout << "edge " << vb << "," << vtb << " does not exist" << std::endl;
      exit(0);
    }
    boost::remove_edge(vsb, vb, g_);
    boost::remove_edge(vb, vtb, g_);
    si_->freeState(stateProperty_[vb]);
    if(DEBUG) std::cout << "rm edge vsb-vb-vtb " << vsb << "," << vb << "," << vtb << " (edges: " << num_edges(g_) << ")" << std::endl;

    boost::clear_vertex(vb, g_);
    boost::remove_vertex(vb, g_);
    Edge eb = boost::add_edge(vsb, vtb, EdgeProperty(ob::Cost(dsb.value()+dtb.value())), g_).first;
    uniteComponents(vsb, vtb);

    const Vertex v1 = boost::source(eb, g_);
    const Vertex v2 = boost::target(eb, g_);
    if(DEBUG) std::cout << "restored edge "<< v1 << "," << v2 << " (edges: " << num_edges(g_) << ")" << std::endl;
  }
  if(isAonEdge){
    boost::remove_edge(vsa, va, g_);
    boost::remove_edge(va, vta, g_);
    si_->freeState(stateProperty_[va]);
    if(DEBUG) std::cout << "rm edge vsa-va-vta " << vsa << "," << va << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;

    boost::clear_vertex(va, g_);
    boost::remove_vertex(va, g_);

    boost::add_edge(vsa, vta, EdgeProperty(ob::Cost(dsa.value()+dta.value())), g_);
    uniteComponents(vsa, vta);

    if(DEBUG) std::cout << "restored edge "<< vsa << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;
  }

  if(DEBUG) std::cout << "graph (edges: " << num_edges(g_) << ")" << std::endl;
  if(DEBUG) std::cout << std::string(80, '-') << std::endl;


  return sol;
}

bool PRMSliceConnect::Connect(const Vertex a, const Vertex b){
  if(previous==nullptr){
    return PRMBasic::Connect(a,b);
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

    const Vertex vsaM0 = associatedVertexSourceProperty_[a];
    const Vertex vsbM0 = associatedVertexSourceProperty_[b];
    const Vertex vtaM0 = associatedVertexTargetProperty_[a];
    const Vertex vtbM0 = associatedVertexTargetProperty_[b];

    //create PWL function between vertices.

    ob::PathPtr sol = previous->GetShortestPathOffsetVertices( qaM0, qbM0, vsaM0, vsbM0, vtaM0, vtbM0);
    if(sol==nullptr){
      std::cout << "nullptr" << std::endl;
      std::cout << vsaM0 << "," << vtaM0 << std::endl;
      std::cout << vsbM0 << "," << vtbM0 << std::endl;
      exit(0);
      return false;
    }
    double D = sol->length();

    og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    std::vector<ob::State *> states = path.getStates();
    if(DEBUG){
      std::cout << "Path states: " << states.size() << std::endl;
    }

    Vertex v0 = a;
    ob::State *s0M0 = states.at(0);
    ob::State *s0M1 = qa;
    for(uint i = 1; i < states.size(); i++)
    {

      ob::State *s1M0 = states.at(i);
      double d = M0->distance(s0M0, s1M0);
      ob::State* s1C1 = C1->allocState();
      C1->getStateSpace()->interpolate(qaC1,qbC1,d/D,s1C1);

      if(DEBUG){
        std::cout << "interpolate[" << i << "/"<< states.size() << "]: " << d << "/" << D << std::endl;
      }

      //mergeStates(ob::State *qM0, ob::State *qC1, ob::State *qM1);
      ob::State* s1M1 = M1->allocState();
      mergeStates(s1M0, s1C1, s1M1);

      C1->freeState(s1C1);

      Vertex v1;
      if(i<states.size()-1){
        v1 = boost::add_vertex(g_);
        stateProperty_[v1] = s1M1;
      }else{
        v1 = b;
      }

      if (si_->checkMotion(s0M1, s1M1))
      {
        boost::add_edge(v0, v1, EdgeProperty(ob::Cost(d)), g_);
        uniteComponents(v0, v1);
      }else{
        C1->freeState(qaC1);
        C1->freeState(qbC1);
        M0->freeState(qaM0);
        M0->freeState(qbM0);
        return false;
      }

      s0M0 = s1M0;

    }

    C1->freeState(qaC1);
    C1->freeState(qbC1);
    M0->freeState(qaM0);
    M0->freeState(qbM0);
    return true;
  }

  return false;
}
