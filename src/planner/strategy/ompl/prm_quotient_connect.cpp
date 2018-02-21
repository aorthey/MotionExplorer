#include "common.h"
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
    assert(PRMprevious->startM_.size() == startM_.size());
    assert(PRMprevious->goalM_.size() == goalM_.size());
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
PRMBasic::Vertex PRMQuotientConnect::CreateNewVertex(ob::State *state)
{
  Vertex m = PRMQuotient::CreateNewVertex(state);
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
  //std::cout << "sampled: " << v1 << "," << v2 << "," << t << std::endl;
  isSampled = true;
  return true;
}

double PRMQuotientConnect::Distance(const Vertex a, const Vertex b) const
{
  if(previous == nullptr){
    return si_->distance(stateProperty_[a], stateProperty_[b]);
  }else{
    og::PRMQuotientConnect *PRMprevious = dynamic_cast<og::PRMQuotientConnect*>(previous);
    if(!PRMprevious->isSampled) return si_->distance(stateProperty_[a], stateProperty_[b]);

    ob::State* qa = stateProperty_[a];
    ob::State* qb = stateProperty_[b];

    ob::State* qaC1 = C1->allocState();
    ob::State* qbC1 = C1->allocState();
    ExtractC1Subspace(qa, qaC1);
    ExtractC1Subspace(qb, qbC1);

    ob::State* qaM0 = M0->allocState();
    ob::State* qbM0 = M0->allocState();
    ExtractM0Subspace(qa, qaM0);
    ExtractM0Subspace(qb, qbM0);

    const Vertex asM0 = associatedVertexSourceProperty_[a];
    const Vertex atM0 = associatedVertexTargetProperty_[a];

    const Vertex bsM0 = associatedVertexSourceProperty_[b];
    const Vertex btM0 = associatedVertexTargetProperty_[b];

    // const double ta = associatedTProperty_[a];
    // const double tb = associatedTProperty_[b];
    // std::cout << "vertices: " << asM0 << "<->" << atM0 << "(" << ta << ") ," << bsM0 << "<->" << btM0 << "(" << tb << ")" << std::endl;

    ob::PathPtr sol = dynamic_cast<PRMQuotientConnect*>(previous)->GetShortestPathOffsetVertices( qaM0, qbM0, asM0, bsM0, atM0, btM0);
    double d0 = +dInf;
    if(sol!=nullptr){
      d0 = sol->length();
    }

    double d1 = C1->distance(qaC1, qbC1);

    // if((bsM0==0 && btM0==0) || (asM0==0 && atM0==0)){
    //   std::cout << "original metric dist: " << M0->distance(qaM0,qbM0) << std::endl;
    //   std::cout << "graph metric dist   : " << d0 << std::endl;
    //   std::cout << "dist: " << d0 <<"+" << d1 << "=" << d0+d1 << std::endl;
    //   std::cout << "num vertices:" << num_vertices(g_) << std::endl;

    //   std::cout << std::string(80, '-') << std::endl;
    //   foreach (Vertex v, boost::vertices(g_))
    //   {
    //     std::cout << "v: " << v << std::endl;
    //     std::cout << "src: " << associatedVertexSourceProperty_[v] << std::endl;
    //     std::cout << "trg: " << associatedVertexTargetProperty_[v] << std::endl;
    //     std::cout << " dt: " << associatedTProperty_[v] << std::endl;
    //     si_->printState(stateProperty_[v]);
    //   }
    //   std::cout << std::string(80, '-') << std::endl;

    //   std::cout << "a: " << a << std::endl;
    //   si_->printState(qa);
    //   std::cout << "b: " << b << std::endl;
    //   si_->printState(qb);
    //   exit(0);
    // }
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

    // uniteComponents(vsa, va);
    // uniteComponents(va, vta);

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

    // uniteComponents(vsb, vb);
    // uniteComponents(vb, vtb);

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
    // uniteComponents(vsb, vtb);

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
    // uniteComponents(vsa, vta);

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
    //#########################################################################
    // Get shortest path on M0, represented by the shortest path on the underlying graph
    //#########################################################################
    og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
    ob::State* qa = stateProperty_[a];
    ob::State* qb = stateProperty_[b];

    ob::State* qaC1 = C1->allocState();
    ob::State* qbC1 = C1->allocState();
    ExtractC1Subspace(qa, qaC1);
    ExtractC1Subspace(qb, qbC1);

    ob::State* qaM0 = M0->allocState();
    ob::State* qbM0 = M0->allocState();
    ExtractM0Subspace(qa, qaM0);
    ExtractM0Subspace(qb, qbM0);

    const Vertex asM0 = associatedVertexSourceProperty_[a];
    const Vertex atM0 = associatedVertexTargetProperty_[a];
    const Vertex bsM0 = associatedVertexSourceProperty_[b];
    const Vertex btM0 = associatedVertexTargetProperty_[b];

    //create PWL function between vertices.
    // std::cout << "CONNECT" << std::endl;
    // const double ta = associatedTProperty_[a];
    // const double tb = associatedTProperty_[b];
    // std::cout << "vertices: " << asM0 << "<->" << atM0 << "(" << ta << ") ," << bsM0 << "<->" << btM0 << "(" << tb << ")" << std::endl;

    PRMprevious->shortestVertexPath_.clear();

    ob::PathPtr sol = PRMprevious->GetShortestPathOffsetVertices( qaM0, qbM0, asM0, bsM0, atM0, btM0);
    double D = sol->length();
    if(D>=dInf){
      std::cout << D << std::endl;
      exit(0);
    }

    og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    std::vector<ob::State *> states = path.getStates();

    //#########################################################################
    // move along vertices of M0 and create new milestones on M1
    //#########################################################################
    //a -> PRMprevious->shortestVertexPath_ ->b

    if(states.size() <= 2){
      //a,b are on the same underlying edge. no new milestones are created,
      //just a single edge between a,b
      return PRMQuotient::Connect(a,b);
    }

    //states has at least one intermediate state

    Vertex v0M1 = a;
    ob::State *s0M0 = states.at(0);
    ob::State *s0M1 = qa;

    double d_graph_distance = 0.0;

    std::vector<Vertex> vpath = PRMprevious->shortestVertexPath_;

    for(uint i = 1; i < states.size(); i++)
    {
      //#########################################################################
      //Compute s1M1
      //s0 last state (startState), s1 is intermediate state
      //#########################################################################
      ob::State *s1M0 = states.at(i);
      Vertex v1M0 = vpath.at(i);

      d_graph_distance += M0->distance(s0M0, s1M0);

      ob::State* s1C1 = C1->allocState();
      C1->getStateSpace()->interpolate(qaC1,qbC1,d_graph_distance/D,s1C1);

      ob::State *s1M1 = M1->allocState();
      mergeStates(s1M0, s1C1, s1M1);

      C1->freeState(s1C1);

      //#########################################################################
      //s0M1 state on M1 at start vertex, s1M1 state on M1 at intermediate vertex
      //#########################################################################

      if (si_->checkMotion(s0M1, s1M1))
      {

        //#########################################################################
        //Create new vertex on M1, associated to state s1M1
        //#########################################################################
        Vertex v1M1;
        if(i<states.size()-1){
          v1M1 = CreateNewVertex(s1M1);
          associatedVertexSourceProperty_[v1M1]=v1M0;
          associatedVertexTargetProperty_[v1M1]=v1M0;
          associatedTProperty_[v1M1]=0;
          nn_->add(v1M1);
          totalNumberOfSamples++;
        }else{
          v1M1 = b;
        }

        //#########################################################################
        //add new edge between v0M1 and v1M1
        //#########################################################################
        double d01 = M1->distance(s0M1, s1M1);
        boost::add_edge(v0M1, v1M1, EdgeProperty(ob::Cost(d01)), g_);
        uniteComponents(v0M1, v1M1);

        //#########################################################################
        //set v0M1 to next state
        //#########################################################################
        v0M1 = v1M1;
        s0M0 = s1M0;
        s0M1 = s1M1;


        ///#DEBUG #################################################
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
        //     for(uint k = 2; k < vpath.size()-1; k++){
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
        //cannot add an edge
        C1->freeState(qaC1);
        C1->freeState(qbC1);
        M0->freeState(qaM0);
        M0->freeState(qbM0);
        M1->freeState(s1M1);
        return false;
      }


    }

    C1->freeState(qaC1);
    C1->freeState(qbC1);
    M0->freeState(qaM0);
    M0->freeState(qbM0);
    return true;
  }
}

//uint PRMQuotientConnect::randomBounceMotion(const ob::StateSamplerPtr &sss, 
//    const Vertex &v, std::vector<ob::State *> &states) const
//{
//  uint steps = states.size();
//  const ob::State *prev = stateProperty_[v];
//  std::pair<ob::State *, double> lastValid;
//  uint j = 0;
//  for (uint i = 0; i < steps; ++i)
//  {
//    sss->sampleUniform(states[j]);
//    lastValid.first = states[j];
//    if (si_->checkMotion(prev, states[j], lastValid) || lastValid.second > std::numeric_limits<double>::epsilon())
//      prev = states[j++];
//  }
//  return j;
//}

