#include "common.h"
#include "prm_quotient_connect.h"
#include "planner/cspace/cspace.h"

#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/PlannerDataGraph.h>
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
  goalBias_ = 0.05;
  epsilon = 0;
  percentageSamplesOnShortestPath = 0.1;
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
bool PRMQuotientConnect::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(previous == nullptr){
    if(!hasSolution && rng_.uniform01() < goalBias_){
      q_random = si_->cloneState(stateProperty_[goalM_.at(0)]);
    }else{
      M1_valid_sampler->sample(q_random);
    }
  }else{
    //Adjusted sampling function: Sampling in G0 x C1
    if(!hasSolution && rng_.uniform01() < goalBias_){
      //goal->sampleGoal(q_random);
      q_random = si_->cloneState(stateProperty_[goalM_.at(0)]);
    }else{
      ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
      base::State *s_C1 = C1->allocState();
      base::State *s_M0 = M0->allocState();

      C1_sampler->sampleUniform(s_C1);
      previous->SampleGraph(s_M0);
      mergeStates(s_M0, s_C1, q_random);

      C1->freeState(s_C1);
      M0->freeState(s_M0);
    }
  }
  return M1->isValid(q_random);
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
  if(epsilon>0) M1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);

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

    ob::PathPtr M1_path = InterpolateGraphConstraint(a, b);
    double d = +dInf;
    if(M1_path!=nullptr){
      d = M1_path->length();
    }
    return d;

    // ob::State* qa = stateProperty_[a];
    // ob::State* qb = stateProperty_[b];

    // ob::State* qaC1 = C1->allocState();
    // ob::State* qbC1 = C1->allocState();
    // ExtractC1Subspace(qa, qaC1);
    // ExtractC1Subspace(qb, qbC1);

    // ob::State* qaM0 = M0->allocState();
    // ob::State* qbM0 = M0->allocState();
    // ExtractM0Subspace(qa, qaM0);
    // ExtractM0Subspace(qb, qbM0);

    // const Vertex asM0 = associatedVertexSourceProperty_[a];
    // const Vertex atM0 = associatedVertexTargetProperty_[a];

    // const Vertex bsM0 = associatedVertexSourceProperty_[b];
    // const Vertex btM0 = associatedVertexTargetProperty_[b];

    // // const double ta = associatedTProperty_[a];
    // // const double tb = associatedTProperty_[b];
    // // std::cout << "vertices: " << asM0 << "<->" << atM0 << "(" << ta << ") ," << bsM0 << "<->" << btM0 << "(" << tb << ")" << std::endl;

    // ob::PathPtr sol = dynamic_cast<PRMQuotientConnect*>(previous)->GetShortestPathOffsetVertices( qaM0, qbM0, asM0, bsM0, atM0, btM0);
    //double d0 = +dInf;
    //if(sol!=nullptr){
    //  d0 = sol->length();
    //}

    //double d1 = C1->distance(qaC1, qbC1);

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
    // C1->freeState(qaC1);
    // C1->freeState(qbC1);
    // M0->freeState(qaM0);
    // M0->freeState(qbM0);

    //return d0 + d1;
  }
}

//@brief qa,qb \in M1. vsa,vsb,vta,vtb are vertices on G0
//the resulting path is an interpolation in M1, such that each point lies in the
//slice of the graph G0

ob::PathPtr PRMQuotientConnect::InterpolateGraphConstraint( const Vertex a, const Vertex b) const
{
  og::PRMQuotientConnect *PRMprevious = dynamic_cast<og::PRMQuotientConnect*>(previous);
  ob::State* sa = stateProperty_[a];
  ob::State* sb = stateProperty_[b];

  ob::State* saC1 = C1->allocState();
  ob::State* sbC1 = C1->allocState();
  ExtractC1Subspace(sa, saC1);
  ExtractC1Subspace(sb, sbC1);

  ob::State* saM0 = M0->allocState();
  ob::State* sbM0 = M0->allocState();

  ExtractM0Subspace(sa, saM0);
  ExtractM0Subspace(sb, sbM0);

  const Vertex asM0 = associatedVertexSourceProperty_[a];
  const Vertex atM0 = associatedVertexTargetProperty_[a];

  const Vertex bsM0 = associatedVertexSourceProperty_[b];
  const Vertex btM0 = associatedVertexTargetProperty_[b];

  ob::PathPtr M0_path = dynamic_cast<PRMQuotientConnect*>(previous)->GetShortestPathOffsetVertices( saM0, sbM0, asM0, bsM0, atM0, btM0);
  std::vector<Vertex> M0_vpath = PRMprevious->shortestVertexPath_;
  double D = M0_path->length();
  if(D>=dInf){
    std::cout << D << std::endl;
    exit(0);
  }
  //Note: shortestVertexPath_ contains all vertices along shortest path

  //###########################################################################
  //interpolate along shortestvertexpath_
  //###########################################################################

  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*M0_path);
  std::vector<ob::State *> M0_spath = gpath.getStates();


  //###########################################################################
  //DEBUG
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "M0 path:" << std::endl;
  for(uint k = 0; k < M0_spath.size(); k++){
    M0->printState(M0_spath.at(k));
  }
  //###########################################################################

  //#########################################################################
  // move along vertices of M0 and create new milestones on M1
  //#########################################################################

  auto M1_path = std::make_shared<PathGeometric>(M1);

  //ob::State *s_prevM1 = sa;
  M1_path->append(sa);

  double d_graph_distance = 0.0;

  for(uint i = 1; i < M0_spath.size(); i++)
  {
    ob::State *s_prevM0 = M0_spath.at(i-1);
    ob::State *s_nextM0 = M0_spath.at(i);

    d_graph_distance += M0->distance(s_prevM0, s_nextM0);

    ob::State* s_nextC1 = C1->allocState();
    C1->getStateSpace()->interpolate(saC1,sbC1,d_graph_distance/D,s_nextC1);

    ob::State *s_nextM1 = M1->allocState();
    mergeStates(s_nextM0, s_nextC1, s_nextM1);

    M0->freeState(s_nextM0);
    C1->freeState(s_nextC1);

    M1_path->append(s_nextM1);
    //s_prevM1 = s_nextM1;
  }

  M0->freeState(saM0);
  M0->freeState(sbM0);
  C1->freeState(saC1);
  C1->freeState(sbC1);


  //###########################################################################
  //DEBUG
  og::PathGeometric gpathM1 = static_cast<og::PathGeometric&>(*M1_path);
  std::vector<ob::State *> M1_spath = gpathM1.getStates();
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "M1 path:" << std::endl;
  for(uint k = 0; k < M1_spath.size(); k++){
    M1->printState(M1_spath.at(k));
  }
  exit(0);
  //###########################################################################
  return M1_path;
}

ob::PathPtr PRMQuotientConnect::GetShortestPathOffsetVertices(const ob::State *qa, const ob::State *qb, 
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
    boost::remove_edge(vsa, vta, g_);
    va = boost::add_vertex(g_);
    stateProperty_[va] = si_->cloneState(qa);

    dsa = opt_->motionCost(stateProperty_[vsa], stateProperty_[va]);
    dta = opt_->motionCost(stateProperty_[va], stateProperty_[vta]);
    boost::add_edge(vsa, va, EdgeProperty(dsa), g_);
    boost::add_edge(va, vta, EdgeProperty(dta), g_);

  }else{
    va = vsa;
    stateProperty_[va] = si_->cloneState(qa);
  }

  if(isBonEdge){
    boost::remove_edge(vsb, vtb, g_);
    vb = boost::add_vertex(g_);
    stateProperty_[vb] = si_->cloneState(qb);

    dsb = opt_->motionCost(stateProperty_[vsb], stateProperty_[vb]);
    dtb = opt_->motionCost(stateProperty_[vb], stateProperty_[vtb]);
    boost::add_edge(vsb, vb, EdgeProperty(dsb), g_);
    boost::add_edge(vb, vtb, EdgeProperty(dtb), g_);
  }else{
    vb = vsb;
    stateProperty_[vb] = si_->cloneState(qb);
  }

  //###########################################################################
  //search in modified graph
  //###########################################################################

  bool same_component = sameComponent(va, vb);
  ob::PathPtr path = nullptr;
  if(same_component){
    path = constructSolution(va, vb);
  }else{
    std::cout << "not same component" << std::endl;
    exit(0);
  }

  //###########################################################################
  //return to former graph
  //###########################################################################

  if(isBonEdge){
    boost::remove_edge(vsb, vb, g_);
    boost::remove_edge(vb, vtb, g_);
    si_->freeState(stateProperty_[vb]);

    boost::clear_vertex(vb, g_);
    boost::remove_vertex(vb, g_);

    ob::Cost db = opt_->motionCost(stateProperty_[vsb], stateProperty_[vtb]);
    boost::add_edge(vsb, vtb, EdgeProperty(db), g_);
  }
  if(isAonEdge){
    boost::remove_edge(vsa, va, g_);
    boost::remove_edge(va, vta, g_);
    si_->freeState(stateProperty_[va]);

    boost::clear_vertex(va, g_);
    boost::remove_vertex(va, g_);

    ob::Cost da = opt_->motionCost(stateProperty_[vsa], stateProperty_[vta]);
    boost::add_edge(vsa, vta, EdgeProperty(da), g_);
  }

  return path;
}

bool PRMQuotientConnect::Connect(const Vertex a, const Vertex b){
  if(previous==nullptr){
    return PRMQuotient::Connect(a,b);
  }else{
    og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);

    ob::PathPtr sol = InterpolateGraphConstraint(a,b);

    og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    std::vector<ob::State *> states = path.getStates();

    Vertex v_prev = a;
    std::vector<Vertex> vpath = PRMprevious->shortestVertexPath_;
    ob::State *s_prev = stateProperty_[a];

    for(uint k = 1; k < states.size()-1; k++){
      Vertex v_next = CreateNewVertex(states.at(k));
      associatedVertexSourceProperty_[v_next]=vpath.at(k);
      associatedVertexTargetProperty_[v_next]=vpath.at(k);
      associatedTProperty_[v_next]=0;
      nn_->add(v_next);
      totalNumberOfSamples++;

      double d = M1->distance(states.at(k-1),states.at(k));
      boost::add_edge(v_prev, v_next, EdgeProperty(ob::Cost(d)), g_);
      uniteComponents(v_prev, v_next);
      v_prev = v_next;
      s_prev = states.at(k);
    }

    //connect last vertex to the final vertex
    double d = M1->distance(s_prev, stateProperty_[b]);
    boost::add_edge(v_prev, b, EdgeProperty(ob::Cost(d)), g_);
    uniteComponents(v_prev, b);

    return true;



    // og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
    // ob::State* qa = stateProperty_[a];
    // ob::State* qb = stateProperty_[b];

    // ob::State* qaC1 = C1->allocState();
    // ob::State* qbC1 = C1->allocState();
    // ExtractC1Subspace(qa, qaC1);
    // ExtractC1Subspace(qb, qbC1);

    // ob::State* qaM0 = M0->allocState();
    // ob::State* qbM0 = M0->allocState();
    // ExtractM0Subspace(qa, qaM0);
    // ExtractM0Subspace(qb, qbM0);

    // const Vertex asM0 = associatedVertexSourceProperty_[a];
    // const Vertex atM0 = associatedVertexTargetProperty_[a];
    // const Vertex bsM0 = associatedVertexSourceProperty_[b];
    // const Vertex btM0 = associatedVertexTargetProperty_[b];


    // // PRMprevious->shortestVertexPath_.clear();

    // // ob::PathPtr sol = PRMprevious->GetShortestPathOffsetVertices( qaM0, qbM0, asM0, bsM0, atM0, btM0);
    // // double D = sol->length();
    // // if(D>=dInf){
    // //   std::cout << D << std::endl;
    // //   exit(0);
    // // }

    // og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    // std::vector<ob::State *> states = path.getStates();

    // //#########################################################################
    // //create PWL function between vertices.
    // //#########################################################################

    // //#########################################################################
    // // move along vertices of M0 and create new milestones on M1
    // //#########################################################################
    // //a -> PRMprevious->shortestVertexPath_ ->b

    // if(states.size() <= 2){
    //   //a,b are on the same underlying edge. no new milestones are created,
    //   //just a single edge between a,b
    //   return PRMQuotient::Connect(a,b);
    // }

    // //states has at least one intermediate state

    // Vertex v0M1 = a;
    // ob::State *s0M0 = states.at(0);
    // ob::State *s0M1 = qa;

    // double d_graph_distance = 0.0;

    // std::vector<Vertex> vpath = PRMprevious->shortestVertexPath_;

    // for(uint i = 1; i < states.size(); i++)
    // {
    //   //#########################################################################
    //   //Compute s1M1
    //   //s0 last state (startState), s1 is intermediate state
    //   //#########################################################################
    //   ob::State *s1M0 = states.at(i);
    //   Vertex v1M0 = vpath.at(i);

    //   d_graph_distance += M0->distance(s0M0, s1M0);

    //   ob::State* s1C1 = C1->allocState();
    //   C1->getStateSpace()->interpolate(qaC1,qbC1,d_graph_distance/D,s1C1);

    //   ob::State *s1M1 = M1->allocState();
    //   mergeStates(s1M0, s1C1, s1M1);

    //   C1->freeState(s1C1);

    //   //#########################################################################
    //   //s0M1 state on M1 at start vertex, s1M1 state on M1 at intermediate vertex
    //   //#########################################################################

    //   if (si_->checkMotion(s0M1, s1M1))
    //   {

    //     //#########################################################################
    //     //Create new vertex on M1, associated to state s1M1
    //     //#########################################################################
    //     Vertex v1M1;
    //     if(i<states.size()-1){
    //       v1M1 = CreateNewVertex(s1M1);
    //       associatedVertexSourceProperty_[v1M1]=v1M0;
    //       associatedVertexTargetProperty_[v1M1]=v1M0;
    //       associatedTProperty_[v1M1]=0;
    //       nn_->add(v1M1);
    //       totalNumberOfSamples++;
    //     }else{
    //       v1M1 = b;
    //     }

        // //#########################################################################
        // //add new edge between v0M1 and v1M1
        // //#########################################################################
        // double d01 = M1->distance(s0M1, s1M1);
        // boost::add_edge(v0M1, v1M1, EdgeProperty(ob::Cost(d01)), g_);
        // uniteComponents(v0M1, v1M1);

        // //#########################################################################
        // //set v0M1 to next state
        // //#########################################################################
        // v0M1 = v1M1;
        // s0M0 = s1M0;
        // s0M1 = s1M1;


        // ///#DEBUG #################################################
        // // if(i>=states.size()-1){
        // //   if(states.size()>5){
        // //     for(uint k = 0; k < states.size(); k++){
        // //       std::cout << k << " :";
        // //       M0->printState(states.at(k));
        // //     }
        // //     std::cout << "VA : " << a << std::endl;
        // //     for(uint k = 0; k < vpath.size(); k++){
        // //       std::cout << "V" << k << " : " << vpath.at(k) << std::endl;
        // //     }
        // //     std::cout << "VB : " << b << std::endl;
        // //     for(uint k = 2; k < vpath.size()-1; k++){
        // //       std::pair<Edge,bool> ek = boost::edge(vpath.at(k-1),vpath.at(k),g_);
        // //       std::cout << "E" << k << "(" << vpath.at(k-1) << "," << vpath.at(k) << ") : " << (ek.second?"existing":"ERROR") << std::endl;
        // //     }
        // //     for(uint k = 0; k < vpath.size(); k++){
        // //       ob::State *sk = stateProperty_[vpath.at(k)];
        // //       std::cout << k << " :";
        // //       M1->printState(sk);
        // //     }
        // //     exit(0);
        // //   }
        // // }
        // ///#DEBUG #################################################
      // }else{
        // //cannot add an edge
        // C1->freeState(qaC1);
        // C1->freeState(qbC1);
        // M0->freeState(qaM0);
        // M0->freeState(qbM0);
        // M1->freeState(s1M1);
        // return false;
      // }


    // }

    // C1->freeState(qaC1);
    // C1->freeState(qbC1);
    // M0->freeState(qaM0);
    // M0->freeState(qbM0);
    // return true;
  }
}

ompl::PDF<og::PRMBasic::Edge> PRMQuotientConnect::GetEdgePDF()
{
  PDF<Edge> pdf;
  double t = rng_.uniform01();
  if(t<percentageSamplesOnShortestPath)
  {
    //shortest path sampling (diminishing return?)
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
    //Random Edge (RE) sampling (suffers from high sampling concentrations at
    //vertices with many incoming edges, not ideal, but fast)
    foreach (Edge e, boost::edges(g_))
    {
      const Vertex v1 = boost::source(e, g_);

      if(sameComponent(v1, startM_.at(0))){
        ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
        pdf.add(e, weight.value());
      }
    }
    ////Random Node Edge (RNE) sampling
    //PDF<Vertex> vpdf;
    //foreach (Vertex v, boost::vertices(g_))
    //{
    //  if(sameComponent(v, startM_.at(0))){
    //    vpdf.add(v,1);
    //  }
    //}
    //Vertex v = vpdf.sample(rng_.uniform01());
    //std::pair<IEIterator, IEIterator> iterators = boost::in_edges(boost::vertex(v, g_), g_);
    //for (IEIterator iter = iterators.first; iter != iterators.second; ++iter)
    //{
    //  //pdf.add(*iter, 1);
    //  ob::Cost weight = get(boost::edge_weight_t(), g_, *iter).getCost();
    //  pdf.add(*iter, weight.value());
    //}
  }
  return pdf;
}

// void PRMBasic::RandomWalk(const Vertex &v, std::vector<ob::State *> &states) 
// {
//   og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
//   uint steps = states.size();
//   const ob::State *s_first = stateProperty_[v];
//   Vertex v_first = v;

//   uint ctr = 0;
//   for (uint i = 0; i < steps; ++i)
//   {

//     //#########################################################################
//     //Sample s_last uniformly from G_{k-1} \times C_k
//     //#########################################################################
//     ob::State *s_last = states.at(ctr);

//     base::State *s_last_C1 = C1->allocState();
//     base::State *s_last_M0 = M0->allocState();

//     C1_sampler->sampleUniform(s_last_C1);
//     previous->SampleGraph(s_last_M0);

//     mergeStates(s_last_M0, s_last_C1, s_last);

//     //#########################################################################
//     //compute interpolation between s_prev and s_last on G_{k-1}
//     // the resulting datastructures are
//     //
//     // std::vector<Vertex> vpath
//     // std::vector<ob::State *> spath
//     //#########################################################################

//     Vertex v_last = addMilestone(s_last);

//     ob::PathPtr sol = InterpolateGraphConstraint(v,b);


// //     const Vertex asM0 = associatedVertexSourceProperty_[v_prev];
// //     const Vertex atM0 = associatedVertexTargetProperty_[v_prev];
// //     const Vertex bsM0 = associatedVertexSourceProperty_[v_last];
// //     const Vertex btM0 = associatedVertexTargetProperty_[v_last];

// //     ob::PathPtr sol = PRMprevious->GetShortestPathOffsetVertices(s_first, s_last, asM0, bsM0, atM0, btM0);

// //     double D = sol->length();

// //     og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
// //     std::vector<ob::State *> spathM0 = path.getStates();
// //     std::vector<Vertex> vpathM0 = PRMprevious->shortestVertexPath_;

// //     //#########################################################################
// //     //move along sol/shortestvertexpath_ until infeasible or target reached.
// //     //#########################################################################

// //     ob::State *s_prevM0 = spathM0.at(0);
// //     ob::State *s_prevM1 = s_first;

// //     ob::State *s_firstC1 = C1->allocState();
// //     ExtractC1Subspace(s_first, s_firstC1);

// //     double d_graph_distance = 0.0;

// //     for(uint i = 1; i < spathM0.size(); i++)
// //     {
// //       //#########################################################################
// //       //Creating s1M1
// //       //
// //       //Given s0M0 and s1M1, extract s0C1 and interpolate towards s_next_C1 to
// //       //obtain s1C1. Then merge them to create s1M1.
// //       //#########################################################################
// //       ob::State *s_nextM0 = states.at(i);
// //       d_graph_distance += M0->distance(s_prevM0, s_nextM0);

// //       ob::State* s_nextC1 = C1->allocState();
// //       C1->getStateSpace()->interpolate(s_firstC1, s_lastC1, d_graph_distance/D, s_nextC1);

// //       ob::State *s_nextM1 = M1->allocState();
// //       mergeStates(s_nextM0, s_nextC1, s_nextM1);

// //       C1->freeState(s_nextC1);

// //       //#########################################################################
// //       //Start at s0M1, move towards s1M1.
// //       //
// //       //If feasible, add vertex at s1M1 to the graph G1
// //       //If infeasible, check if we made at least a distance of epsilon progress
// //       //towards s1M1. If yes, then add the last valid state as a vertex to graph
// //       //G1. If no, then break.
// //       //#########################################################################

// //       std::pair<ob::State *, double> lastValid;
// //       lastValid.first = s_next;
// //       if(si_->checkMotion(s0M1, s1M1, lastValid)){

// //         Vertex v_next = CreateNewVertex(s_next);

// //         EdgeProperty properties(opt_->motionCost(stateProperty_[v_prev], stateProperty_[v_next]));
// //         boost::add_edge(v_prev, v_next, properties, g_);
// //         uniteComponents(v_prev, v_next);
// //         nn_->add(v_next);

// //         v_prev = v_next;
// //         s_prev = s_next;
// //       }else{
// //         if(lastValid.second > std::numeric_limits<double>::epsilon()){
// //           addMilestone(lastValid.first);
// //         }else{
// //           //no progress made, return
// //         }
// //         break;
// //       }

// //     M0->freeState(s_next_M0);
// //     C1->freeState(s_next_C1);
//   }
// }

