#include "prm_quotient_connect.h"
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
#define DEBUG 1

namespace ompl
{
  namespace magic
  {
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
    static const double ROADMAP_BUILD_TIME = 0.01;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}

PRMQuotientConnect::PRMQuotientConnect(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  PRMQuotient(si, previous_)
{
  setName("PRMQuotientConnect"+to_string(id));
}

PRMQuotientConnect::~PRMQuotientConnect(){
}

void PRMQuotientConnect::setup()
{
  og::PRMQuotient::setup();
  nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return PRMQuotientConnect::Distance(a,b);
                           });
}

void PRMQuotientConnect::Init()
{
  PRMQuotient::Init();

  og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
  if(PRMprevious!=nullptr){
    for(uint k = 0; k < startM_.size(); k++){
      associatedVertexSourceProperty_[startM_.at(k)] = PRMprevious->startM_.at(k);
      associatedVertexTargetProperty_[startM_.at(k)] = PRMprevious->startM_.at(k);
      associatedTProperty_[startM_.at(k)] = 0;
    }
    for(uint k = 0; k < goalM_.size(); k++){
      associatedVertexSourceProperty_[goalM_.at(k)] = PRMprevious->goalM_.at(k);
      associatedVertexTargetProperty_[goalM_.at(k)] = PRMprevious->goalM_.at(k);
      associatedTProperty_[goalM_.at(k)] = 0;
    }
  }
}

og::PRMBasic::Vertex PRMQuotientConnect::addMilestone(base::State *state)
{
  Vertex m = PRMQuotient::addMilestone(state);
    
  og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
  if(PRMprevious != nullptr && PRMprevious->isSampled){
    associatedVertexSourceProperty_[m] = PRMprevious->lastSourceVertexSampled;
    associatedVertexTargetProperty_[m] = PRMprevious->lastTargetVertexSampled;
    associatedTProperty_[m] = PRMprevious->lastTSampled;
  }
  return m;
}

bool PRMQuotientConnect::SampleGraph(ob::State *q_random_graph)
{
  PDF<Edge> pdf = GetEdgePDF();

  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, g_);
  const Vertex v2 = boost::target(e, g_);
  const ob::State *from = stateProperty_[v1];
  const ob::State *to = stateProperty_[v2];

  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);

  simpleSampler_->sampleGaussian(q_random_graph, q_random_graph, epsilon);

  lastSourceVertexSampled = v1;
  lastTargetVertexSampled = v2;
  lastTSampled = t;
  isSampled = true;
  return true;
}

double PRMQuotientConnect::Distance(const Vertex a, const Vertex b) const
{
  if(previous == nullptr){
    return PRMQuotient::Distance(a,b);
  }else{
    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();

    ob::State* qa = stateProperty_[a];
    ob::State* qb = stateProperty_[b];

    ob::State* qaC1 = C1->allocState();
    ob::State* qaM0 = M0->allocState();

    ob::State* qbC1 = C1->allocState();
    ob::State* qbM0 = M0->allocState();

    ExtractC1Subspace(qa, qaC1);
    ExtractC1Subspace(qb, qbC1);
    ExtractM0Subspace(qa, qaM0);
    ExtractM0Subspace(qb, qbM0);

    const Vertex vsaM0 = associatedVertexSourceProperty_[a];
    const Vertex vsbM0 = associatedVertexSourceProperty_[b];
    const Vertex vtaM0 = associatedVertexTargetProperty_[a];
    const Vertex vtbM0 = associatedVertexTargetProperty_[b];

    ob::PathPtr sol = dynamic_cast<PRMQuotientConnect*>(previous)->GetShortestPathOffsetVertices( qaM0, qbM0, vsaM0, vsbM0, vtaM0, vtbM0);
    double d0 = +dInf;
    if(sol!=nullptr){
      d0 = sol->length();
    }

    double d1 = C1->distance(qaC1, qbC1);

    C1->freeState(qaC1);
    C1->freeState(qbC1);
    M0->freeState(qaM0);
    M0->freeState(qbM0);

    return d0 + d1;
  }
}

ob::PathPtr PRMQuotientConnect::GetShortestPathOffsetVertices( const ob::State *qa, const ob::State *qb, 
  const Vertex vsa, const Vertex vsb, const Vertex vta, const Vertex vtb)
{
  //###########################################################################
  //construct modified graph
  //###########################################################################
  Vertex va;
  Vertex vb;

  // Input: two edges ea=(vsa --- vta)   and    eb=(vsb --- vtb)
  // 
  //  Four cases:
  //  (1) vsa=vta               :  (vsa) and (vsb --- vtb)
  //  (2) vsb=vtb               :  (vsa --- vta) and (vsb)
  //  (3) vsa=vta and vsb=vtb   :  (vsa) and (vsb)
  //  (4) vsa!=vta and vsb!=vtb :  (vsa --- vta) and (vsb --- vtb)
  //

  //  Case(4): check if ea==eb
  if(vsa!=vta && vsb!=vtb){
    const Edge ea = boost::edge(vsa,vta,g_).first;
    const Edge eb = boost::edge(vsb,vtb,g_).first;
    if(ea==eb){
      ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
      return path;
    }
  }
  //  Case(1): check if vsb=vsa or vsb=vta
  //  Case(2): check if vsa=vsb or vsa=vtb
  if( (vsa==vta && vsa==vsb) ||
      (vsa==vta && vsa==vtb) ||
      (vsb==vtb && vsb==vsa) ||
      (vsb==vtb && vsb==vta)){
    ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
    return path;
  }
  //if(DEBUG) std::cout << std::string(80, '-') << std::endl;
  //if(DEBUG) std::cout << "graph (edges: " << num_edges(g_) << ")" << std::endl;

  //  Check cases:
  // (1) isAonEdge  && !isBonEdge
  // (2) !isAonEdge && isBonEdge
  // (3) !isAonEdge && !isBonEdge
  // (4) isAonEdge  && isBonEdge

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
    si_->copyState(stateProperty_[va], qa);

    //if(DEBUG) std::cout << "rm edge "<< vsa << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;

    dsa = opt_->motionCost(stateProperty_[vsa], stateProperty_[va]);
    dta = opt_->motionCost(stateProperty_[va], stateProperty_[vta]);
    boost::add_edge(vsa, va, EdgeProperty(dsa), g_);
    boost::add_edge(va, vta, EdgeProperty(dta), g_);

    uniteComponents(vsa, va);
    uniteComponents(va, vta);

    //if(DEBUG) std::cout << "added edge vsa-va-vta " << vsa << "," << va << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;
  }else{
    va = vsa;
    stateProperty_[va] = si_->cloneState(qa);
  }

  if(isBonEdge){
    if(!boost::edge(vsb,vtb,g_).second){
      std::cout << "edge vsa-vta does not exist" << std::endl;
      exit(0);
    }
    boost::remove_edge(vsb, vtb, g_);
    vb = add_vertex(g_);
    stateProperty_[vb] = si_->cloneState(qb);
    //if(DEBUG) std::cout << "rm edge "<< vsb << "," << vtb << " (edges: " << num_edges(g_) << ")" << std::endl;

    dsb = opt_->motionCost(stateProperty_[vsb], stateProperty_[vb]);
    dtb = opt_->motionCost(stateProperty_[vb], stateProperty_[vtb]);
    boost::add_edge(vsb, vb, EdgeProperty(dsb), g_);
    boost::add_edge(vb, vtb, EdgeProperty(dtb), g_);
    uniteComponents(vsb, vb);
    uniteComponents(vb, vtb);

    //if(DEBUG) std::cout << "added edge vsb-vb-vtb " << vsb << "," << vb << "," << vtb << " (edges: " << num_edges(g_) << ")" << std::endl;
  }else{
    vb = vsb;
    stateProperty_[vb] = si_->cloneState(qb);
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
    //if(DEBUG) std::cout << "rm edge vsb-vb-vtb " << vsb << "," << vb << "," << vtb << " (edges: " << num_edges(g_) << ")" << std::endl;

    boost::clear_vertex(vb, g_);
    boost::remove_vertex(vb, g_);
    boost::add_edge(vsb, vtb, EdgeProperty(ob::Cost(dsb.value()+dtb.value())), g_).first;
    uniteComponents(vsb, vtb);

    //const Vertex v1 = boost::source(eb, g_);
    //const Vertex v2 = boost::target(eb, g_);
    //if(DEBUG) std::cout << "restored edge "<< v1 << "," << v2 << " (edges: " << num_edges(g_) << ")" << std::endl;
  }
  if(isAonEdge){
    boost::remove_edge(vsa, va, g_);
    boost::remove_edge(va, vta, g_);
    si_->freeState(stateProperty_[va]);
    //if(DEBUG) std::cout << "rm edge vsa-va-vta " << vsa << "," << va << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;

    boost::clear_vertex(va, g_);
    boost::remove_vertex(va, g_);

    boost::add_edge(vsa, vta, EdgeProperty(ob::Cost(dsa.value()+dta.value())), g_);
    uniteComponents(vsa, vta);

    //if(DEBUG) std::cout << "restored edge "<< vsa << "," << vta << " (edges: " << num_edges(g_) << ")" << std::endl;
  }

  //if(DEBUG) std::cout << "graph (edges: " << num_edges(g_) << ")" << std::endl;
  //if(DEBUG) std::cout << std::string(80, '-') << std::endl;

  return sol;
}

bool PRMQuotientConnect::Connect(const Vertex a, const Vertex b){
  if(previous==nullptr){
    return PRMQuotient::Connect(a,b);
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

    ob::PathPtr sol = dynamic_cast<PRMQuotientConnect*>(previous)->GetShortestPathOffsetVertices( qaM0, qbM0, vsaM0, vsbM0, vtaM0, vtbM0);
    if(sol==nullptr){
      std::cout << "nullptr" << std::endl;
      std::cout << vsaM0 << "," << vtaM0 << std::endl;
      std::cout << vsbM0 << "," << vtbM0 << std::endl;
      exit(0);
    }
    double D = sol->length();
    if(D>=dInf){
      std::cout << D << std::endl;
      exit(0);
    }

    og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    std::vector<ob::State *> states = path.getStates();

    Vertex v0 = a;
    Vertex v1 = a;
    ob::State *s0M0 = states.at(0);
    ob::State *s0M1 = qa;
    ob::State *s1M0 = states.at(1);

    double d_graph_distance = 0;

    std::vector<Vertex> vpath;
    vpath.push_back(v0);
    for(uint i = 1; i < states.size(); i++)
    {

      s1M0 = states.at(i);
      d_graph_distance += M0->distance(s0M0, s1M0);

      ob::State* s1C1 = C1->allocState();
      C1->getStateSpace()->interpolate(qaC1,qbC1,d_graph_distance/D,s1C1);

      ob::State *s1M1 = M1->allocState();
      mergeStates(s1M0, s1C1, s1M1);
      C1->freeState(s1C1);

      //s0M0 contains last vertex on M0, s1M0 contains current vertex on M0
      //s0M1 contains last vertex, s1M1 contains new vertex to be added

      if (si_->checkMotion(s0M1, s1M1))
      {
        //v0 = v1;
        //stateProperty_[v0] = M1->cloneState(s0M1);

        if(i<states.size()-1){
          v1 = CreateNewVertex(s1M1);
          nn_->add(v1);
          totalNumberOfSamples++;
        }else{
          v1 = b;
        }
        double d01 = M1->distance(s0M1, s1M1);
        boost::add_edge(v0, v1, EdgeProperty(ob::Cost(d01)), g_);
        uniteComponents(v0, v1);
        v0 = v1;
        ///#DEBUG #################################################
        // vpath.push_back(v1);
        // if(i>=states.size()-1){
        //   if(states.size()>5){
        //     for(uint k = 0; k < states.size(); k++){
        //       std::cout << k << " :";
        //       M0->printState(states.at(k));
        //     }
        //     std::cout << "VA : " << a << std::endl;
        //     for(uint k = 0; k < vpath.size(); k++){
        //       std::cout << "V" << k << " : " << vpath.at(k) << std::endl;
        //     }
        //     std::cout << "VB : " << b << std::endl;
        //     for(uint k = 1; k < vpath.size(); k++){

        //       std::pair<Edge,bool> ek = boost::edge(vpath.at(k-1),vpath.at(k),g_);
        //       std::cout << "E" << k << "(" << vpath.at(k-1) << "," << vpath.at(k) << ") : " << (ek.second?"existing":"ERROR") << std::endl;
        //     }
        //     for(uint k = 0; k < vpath.size(); k++){
        //       ob::State *sk = stateProperty_[vpath.at(k)];
        //       std::cout << k << " :";
        //       M1->printState(sk);
        //     }
        //     exit(0);
        //   }
        // }
        ///#DEBUG #################################################
      }else{
        C1->freeState(qaC1);
        C1->freeState(qbC1);
        M0->freeState(qaM0);
        M0->freeState(qbM0);
        return false;
      }

      s0M0 = s1M0;
      s0M1 = s1M1;

    }

    C1->freeState(qaC1);
    C1->freeState(qbC1);
    M0->freeState(qaM0);
    M0->freeState(qbM0);
    return true;
  }
}

uint PRMQuotientConnect::randomBounceMotion(const ob::StateSamplerPtr &sss, 
    const Vertex &v, std::vector<ob::State *> &states) const
{
  uint steps = states.size();
  const ob::State *prev = stateProperty_[v];
  std::pair<ob::State *, double> lastValid;
  uint j = 0;
  for (uint i = 0; i < steps; ++i)
  {
    sss->sampleUniform(states[j]);
    lastValid.first = states[j];
    if (si_->checkMotion(prev, states[j], lastValid) || lastValid.second > std::numeric_limits<double>::epsilon())
      prev = states[j++];
  }
  return j;
}

