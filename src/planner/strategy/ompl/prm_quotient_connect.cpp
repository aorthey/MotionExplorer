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
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>

using namespace og;
using namespace ob;

#define foreach BOOST_FOREACH
namespace ompl
{
  namespace magic
  {
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}

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
  og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
  if(PRMprevious==nullptr){
    PRMQuotient::setup();
  }

  if (!nn_){
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                             {
                               return PRMQuotientConnect::Distance(a,b);
                             });
  }
  if (!connectionStrategy_){
    connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
  }

  if (pdef_){
    Planner::setup();
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
    }
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr){
      OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
      exit(0);
    }

    //we assume here that there is one single start and that we can sample one
    //single goal state associated with the problem. @TODO: make this more
    //general

    if(const ob::State *st = pis_.nextStart()){
      Vertex m = CreateNewVertex(si_->cloneState(st));
      associatedVertexSourceProperty_[m] = PRMprevious->startM_.at(0);
      associatedVertexTargetProperty_[m] = PRMprevious->startM_.at(0);
      associatedTProperty_[m] = 0;
      std::cout << "start vertex: " << m << " associated:" << PRMprevious->startM_.at(0) << std::endl;
      ConnectVertexToNeighbors(m);
      startM_.push_back(m);
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
      const ob::State *gl = pis_.nextGoal();
      if (gl != nullptr){
        Vertex m = CreateNewVertex(si_->cloneState(gl));
        associatedVertexSourceProperty_[m] = PRMprevious->goalM_.at(0);
        associatedVertexTargetProperty_[m] = PRMprevious->goalM_.at(0);
        associatedTProperty_[m] = 0;
        std::cout << "goal vertex: " << m << " associated:" << PRMprevious->goalM_.at(0) << std::endl;
        ConnectVertexToNeighbors(m);
        goalM_.push_back(m);
      }
    }
    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
  }else{
    setup_ = false;
  }
}

void PRMQuotientConnect::Init()
{
  PRMQuotient::Init();

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
    M1_valid_sampler->sample(q_random);
    //if(!hasSolution && rng_.uniform01() < goalBias_){
    //  //q_random = si_->cloneState(stateProperty_[goalM_.at(0)]);
    //}else{
    //  M1_valid_sampler->sample(q_random);
    //}
  }else{
    //Adjusted sampling function: Sampling in G0 x C1
    //if(!hasSolution && rng_.uniform01() < goalBias_){
    //  //goal->sampleGoal(q_random);
    //  //q_random = si_->cloneState(stateProperty_[goalM_.at(0)]);
    //}else{
      ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
      base::State *s_C1 = C1->allocState();
      base::State *s_M0 = M0->allocState();

      C1_sampler->sampleUniform(s_C1);
      previous->SampleGraph(s_M0);
      mergeStates(s_M0, s_C1, q_random);

      C1->freeState(s_C1);
      M0->freeState(s_M0);
    //}
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

  lastSourceVertexSampled = v1;
  lastTargetVertexSampled = v2;
  lastTSampled = t;
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

    ob::PathPtr M1_path = InterpolateM1GraphConstraint(a, b);
    double d = +dInf;
    if(M1_path){
      d = M1_path->length();
    }

    //double dM1 = M1->distance(stateProperty_[a],stateProperty_[b]);
    //std::cout << "d_graphM1 = " << d   << std::endl;
    //std::cout << "d_M1      = " << dM1 << std::endl;
    //assert(dM1<=d);

    return d;
  }
}

bool PRMQuotientConnect::Connect(const Vertex a, const Vertex b){
  if(previous==nullptr){
    return PRMQuotient::Connect(a,b);
  }else{
    og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);

    ob::PathPtr sol = InterpolateM1GraphConstraint(a,b);
    if(!sol){
      return false;
    }

    og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    std::vector<ob::State *> spathM1 = path.getStates();
    std::vector<Vertex> vpathM0 = PRMprevious->shortestVertexPath_;

    Vertex v_prevM1 = a;
    ob::State *s_prevM1 = spathM1.at(0);

    for(uint k = 1; k < spathM1.size()-1; k++){
      if(!si_->checkMotion(spathM1.at(k-1),spathM1.at(k)))
        return false;

      Vertex v_nextM1 = CreateNewVertex(M1->cloneState(spathM1.at(k)));
      associatedVertexSourceProperty_[v_nextM1]=vpathM0.at(k);
      associatedVertexTargetProperty_[v_nextM1]=vpathM0.at(k);
      associatedTProperty_[v_nextM1]=0;
      nn_->add(v_nextM1);
      totalNumberOfSamples++;

      double dk = M1->distance(spathM1.at(k-1),spathM1.at(k));
      boost::add_edge(v_prevM1, v_nextM1, EdgeProperty(ob::Cost(dk)), g_);
      uniteComponents(v_prevM1, v_nextM1);
      v_prevM1 = v_nextM1;
      s_prevM1 = spathM1.at(k);
    }

    //connect last vertex to the final vertex
    double d_last = M1->distance(s_prevM1, stateProperty_[b]);
    boost::add_edge(v_prevM1, b, EdgeProperty(ob::Cost(d_last)), g_);
    uniteComponents(v_prevM1, b);

    return true;

  }
}


//@brief qa,qb \in M1. vsa,vsb,vta,vtb are vertices on G0
//the resulting path is an interpolation in M1, such that each point lies in the
//slice of the graph G0

ob::PathPtr PRMQuotientConnect::InterpolateM1GraphConstraint( const Vertex a, const Vertex b) const
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

  //std::cout << "associated vertices: " << asM0 << "," << atM0 << "|" << bsM0 << "," << btM0 << "." << std::endl;
  ob::PathPtr M0_path = PRMprevious->GetShortestPathOffsetVertices( saM0, sbM0, asM0, bsM0, atM0, btM0);

  if(!M0_path) return nullptr;

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

  //#########################################################################
  // move along vertices of M0 and create new milestones on M1
  //#########################################################################

  auto M1_path = std::make_shared<PathGeometric>(M1);

  M1_path->append(sa);

  double d_graph_distance = 0.0;

  for(uint i = 1; i < M0_spath.size(); i++)
  {
    ob::State *s_prevM0 = M0_spath.at(i-1);
    ob::State *s_nextM0 = M0_spath.at(i);

    d_graph_distance += M0->distance(s_prevM0, s_nextM0);

    ob::State* s_nextC1 = C1->allocState();
    C1->getStateSpace()->interpolate(saC1, sbC1, d_graph_distance/D,s_nextC1);

    ob::State *s_nextM1 = M1->allocState();
    mergeStates(s_nextM0, s_nextC1, s_nextM1);
    C1->freeState(s_nextC1);

    M1_path->append(s_nextM1);
  }

  M0->freeState(saM0);
  M0->freeState(sbM0);
  C1->freeState(saC1);
  C1->freeState(sbC1);

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
      shortestVertexPath_.clear();
      shortestVertexPath_.push_back(vsa);
      shortestVertexPath_.push_back(vta);
      return path;
    }
  }
  //  Case(1): check if vsb=vsa or vsb=vta
  //  Case(2): check if vsa=vsb or vsa=vtb
  if(vsa==vta){
    if(vsa==vsb){
      ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
      shortestVertexPath_.clear();
      shortestVertexPath_.push_back(vsa);
      shortestVertexPath_.push_back(vtb);
      return path;
    }else if(vsa==vtb){
      ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
      shortestVertexPath_.clear();
      shortestVertexPath_.push_back(vsa);
      shortestVertexPath_.push_back(vsb);
      return path;
    }
  }
  if(vsb==vtb){
    if(vsb==vsa){
      ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
      shortestVertexPath_.clear();
      shortestVertexPath_.push_back(vta);
      shortestVertexPath_.push_back(vsb);
      return path;
    }else if(vsb==vta){
      ob::PathPtr path = std::make_shared<og::PathGeometric>(si_, qa, qb);
      shortestVertexPath_.clear();
      shortestVertexPath_.push_back(vsa);
      shortestVertexPath_.push_back(vsb);
      return path;
    }
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
    //std::cout << "ConstructSolution: " << va << "<->" << vb << std::endl;
    path = constructSolution(va, vb);
  }else{
    std::cout << "WARNING:" << va << " and " << vb << " are not in same component" << std::endl;
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
      if(onShortestPath_[v1] && onShortestPath_[v2])
      {
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

void PRMQuotientConnect::RandomWalk(const Vertex &v) 
{
  og::PRMQuotientConnect *PRMprevious = static_cast<og::PRMQuotientConnect*>(previous);
  if(previous == nullptr){
    return PRMQuotient::RandomWalk(v);
  }

  //const ob::State *s_first = stateProperty_[v];
  //std::cout << "input state:" << std::endl;
  //M1->printState(s_first);

  Vertex v_first = v;

  for (uint i = 0; i < 1; ++i)
  {

    //#########################################################################
    //Sample s_last uniformly from G_{k-1} \times C_k
    //#########################################################################

    ob::State *s_last = xstates[0];

    base::State *s_last_C1 = C1->allocState();
    base::State *s_last_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_last_C1);
    previous->SampleGraph(s_last_M0);

    mergeStates(s_last_M0, s_last_C1, s_last);

    M0->freeState(s_last_M0);
    C1->freeState(s_last_C1);

    //#########################################################################
    //compute interpolation between s_prev and s_last on G_{k-1}
    // the resulting datastructures are
    //
    // std::vector<Vertex> vpath
    // std::vector<ob::State *> spath
    //#########################################################################

    Vertex v_last = addMilestone(s_last);
    associatedVertexSourceProperty_[v_last]=PRMprevious->lastSourceVertexSampled;
    associatedVertexTargetProperty_[v_last]=PRMprevious->lastTargetVertexSampled;
    associatedTProperty_[v_last]=PRMprevious->lastTSampled;

    ob::PathPtr solM1 = InterpolateM1GraphConstraint(v_first, v_last);
    if(!solM1){
      std::cout << "no path towards sampled configuration" << std::endl;
      exit(0);
    }

    og::PathGeometric pathM1 = static_cast<og::PathGeometric&>(*solM1);

    std::vector<ob::State *> spathM1 = pathM1.getStates();
    std::vector<Vertex> vpathM0 = PRMprevious->shortestVertexPath_;

    //#########################################################################
    //move along sol/shortestvertexpath_ until infeasible or target reached.
    //#########################################################################

    Vertex v_prevM1 = v_first;
    ob::State *s_prevM1 = spathM1.at(0);

    bool advanced = false;
    for(uint k = 1; k < spathM1.size(); k++){
      ob::State *s_nextM1 = spathM1.at(k);
      std::pair<ob::State *, double> lastValid;
      lastValid.first = s_nextM1;

      //note: lastvalid contains (state,n), whereby state is the last state
      //still feasible on the straight line between s_prev and s_next. n is a
      //number in [0,1] showing how much progress has been made on the edge.

      if(!si_->checkMotion(s_prevM1, s_nextM1, lastValid)){
        if(lastValid.second > std::numeric_limits<double>::epsilon()){
          advanced = true;
          Vertex v_nextM1 = CreateNewVertex(lastValid.first);
          associatedVertexSourceProperty_[v_nextM1]=vpathM0.at(k);
          associatedVertexTargetProperty_[v_nextM1]=vpathM0.at(k);
          associatedTProperty_[v_nextM1]=0;
          totalNumberOfSamples++;

          double dl = M1->distance(lastValid.first, s_prevM1);
          boost::add_edge(v_prevM1, v_nextM1, EdgeProperty(ob::Cost(dl)), g_);
          uniteComponents(v_prevM1, v_nextM1);

          nn_->add(v_nextM1);
          v_prevM1 = v_nextM1;
          s_prevM1 = lastValid.first;
        }else{
          //no progress made, return
        }
        break;
      }else{
        advanced = true;
        Vertex v_nextM1;
        if(k==spathM1.size()-1){
          //last vertex reached
          v_nextM1 = v_last;
        }else{
          v_nextM1 = CreateNewVertex(s_nextM1);
          associatedVertexSourceProperty_[v_nextM1]=vpathM0.at(k);
          associatedVertexTargetProperty_[v_nextM1]=vpathM0.at(k);
          associatedTProperty_[v_nextM1]=0;
          nn_->add(v_nextM1);
          totalNumberOfSamples++;
        }

        double dk = M1->distance(s_prevM1, s_nextM1);
        boost::add_edge(v_prevM1, v_nextM1, EdgeProperty(ob::Cost(dk)), g_);
        uniteComponents(v_prevM1, v_nextM1);

        v_prevM1 = v_nextM1;
        s_prevM1 = s_nextM1;
      }
    }

    if(advanced){
      v_first = v_prevM1;
    }

  }
  //exit(0);
}

