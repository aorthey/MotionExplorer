#include "common.h"
#include "qmp_connect.h"
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
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 10;
  }
}

QMPConnect::QMPConnect(const ob::SpaceInformationPtr &si, Quotient *previous_ ):
  QMP(si, previous_)
{
  setName("QMPConnect"+to_string(id));
}

QMPConnect::~QMPConnect(){
}

void QMPConnect::clear(){
  QMP::clear();
  lastSourceVertexSampled = -1;
  lastTargetVertexSampled = -1;
  lastTSampled = -1;
  isSampled = false;
  samplesOnShortestPath = 0;
}

void QMPConnect::setup()
{
  og::QMPConnect *quotient_previous = static_cast<og::QMPConnect*>(previous);
  if(quotient_previous==nullptr){
    QMP::setup();
  }
  if (!nn_){
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                             {
                               return QMPConnect::Distance(a,b);
                             });
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
      G[m].associated_source = quotient_previous->startM_.at(0);
      G[m].associated_target = quotient_previous->startM_.at(0);
      G[m].associated_t = 0;
      // G[m].associated_source = 
      // G[m].associated_target = quotient_previous->startM_.at(0);
      // G[m].associated_t = 0;
      //std::cout << "start vertex: " << m << " associated:" << quotient_previous->startM_.at(0) << std::endl;
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
        //G[m].associated_source = quotient_previous->goalM_.at(0);
        //G[m].associated_target = quotient_previous->goalM_.at(0);
        //G[m].associated_t = 0;
        G[m].associated_source = quotient_previous->goalM_.at(0);
        G[m].associated_target = quotient_previous->goalM_.at(0);
        G[m].associated_t = 0;
        //std::cout << "goal vertex: " << m << " associated:" << quotient_previous->goalM_.at(0) << std::endl;
        ConnectVertexToNeighbors(m);
        goalM_.push_back(m);
      }
    }
    unsigned long int nrStartStates = boost::num_vertices(G);
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
  }else{
    setup_ = false;
  }
}

void QMPConnect::Init()
{
  QMP::Init();

}
QuotientGraph::Vertex QMPConnect::CreateNewVertex(ob::State *state)
{
  Vertex m = BaseT::CreateNewVertex(state);
  og::QMPConnect *quotient_previous = static_cast<og::QMPConnect*>(previous);
  if(quotient_previous != nullptr && quotient_previous->isSampled){
    G[m].associated_source = quotient_previous->lastSourceVertexSampled;
    G[m].associated_target = quotient_previous->lastTargetVertexSampled;
    G[m].associated_t = quotient_previous->lastTSampled;
  }
  return m;
}
bool QMPConnect::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(previous == nullptr){
    //return M1_valid_sampler->sample(q_random);
    if(!hasSolution && rng_.uniform01() < goalBias_){
      q_random = si_->cloneState(G[goalM_.at(0)].state);//stateProperty_[goalM_.at(0)]);
      //goal->sampleGoal(q_random);
    }else{
      M1_valid_sampler->sample(q_random);
    }
  }else{
    //Adjusted sampling function: Sampling in G0 x C1
    if(!hasSolution && rng_.uniform01() < goalBias_){
      //goal->sampleGoal(q_random);
      q_random = si_->cloneState(G[goalM_.at(0)].state);//stateProperty_[goalM_.at(0)]);
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

bool QMPConnect::SampleGraph(ob::State *q_random_graph)
{
  PDF<Edge> pdf = GetEdgePDF();

  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, G);
  const Vertex v2 = boost::target(e, G);
  const ob::State *from = G[v1].state;//G[v1].state;
  const ob::State *to = G[v2].state;//G[v2].state;

  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);

  if(epsilon>0) M1_sampler->sampleGaussian(q_random_graph, q_random_graph, epsilon);
  //if(epsilon>0) M1_sampler->sampleUniformNear(q_random_graph, q_random_graph, epsilon);

  lastSourceVertexSampled = v1;
  lastTargetVertexSampled = v2;
  lastTSampled = t;
  isSampled = true;
  return true;
}

double QMPConnect::Distance(const Vertex a, const Vertex b) const
{
  if(previous == nullptr){
    //return si_->distance(G[a].state, G[b].state);
    return si_->distance(G[a].state, G[b].state);
  }else{

    og::QMPConnect *quotient_previous = dynamic_cast<og::QMPConnect*>(previous);
    if(!quotient_previous->isSampled) return si_->distance(G[a].state, G[b].state);

    ob::PathPtr M1_path = InterpolateM1GraphConstraint(a, b);
    double d = +dInf;
    if(M1_path){
      d = M1_path->length();
    }

    //double dM1 = M1->distance(G[a].state,G[b].state);
    //std::cout << "d_graphM1 = " << d   << std::endl;
    //std::cout << "d_M1      = " << dM1 << std::endl;
    //assert(dM1<=d);

    return d;
  }
}

bool QMPConnect::Connect(const Vertex a, const Vertex b){

  if(previous==nullptr){
    return QMP::Connect(a,b);
  }else{
    og::QMPConnect *quotient_previous = static_cast<og::QMPConnect*>(previous);

    ob::PathPtr sol = InterpolateM1GraphConstraint(a,b);
    if(!sol){
      return false;
    }

    og::PathGeometric path = static_cast<og::PathGeometric&>(*sol);
    std::vector<ob::State *> spathM1 = path.getStates();
    std::vector<Vertex> vpathM0 = quotient_previous->shortestVertexPath_;

    Vertex v_prevM1 = a;
    ob::State *s_prevM1 = spathM1.at(0);

    for(uint k = 1; k < spathM1.size(); k++){
      if(!si_->isValid(spathM1.at(k-1))) return false;
      if(!si_->checkMotion(spathM1.at(k-1),spathM1.at(k))) return false;

      Vertex v_nextM1;
      if(k==spathM1.size()-1){
        v_nextM1 = b;
      }else{
        v_nextM1 = CreateNewVertex(spathM1.at(k));
        G[v_nextM1].associated_source = vpathM0.at(k);
        G[v_nextM1].associated_target = vpathM0.at(k);
        G[v_nextM1].associated_t = 0;
        //G[v_nextM1].associated_source=vpathM0.at(k);
        //G[v_nextM1].associated_target=vpathM0.at(k);
        //G[v_nextM1].associated_t=0;
        nn_->add(v_nextM1);
        totalNumberOfSamples++;
      }

      double dk = M1->distance(s_prevM1, spathM1.at(k));
      boost::add_edge(v_prevM1, v_nextM1, EdgeInternalState(ob::Cost(dk)), G);
      uniteComponents(v_prevM1, v_nextM1);
      v_prevM1 = v_nextM1;
      s_prevM1 = spathM1.at(k);
    }

    return true;

  }
}


//@brief qa,qb \in M1. vsa,vsb,vta,vtb are vertices on G0
//the resulting path is an interpolation in M1, such that each point lies in the
//slice of the graph G0

ob::PathPtr QMPConnect::InterpolateM1GraphConstraint( const Vertex a, const Vertex b) const
{
  og::QMPConnect *quotient_previous = dynamic_cast<og::QMPConnect*>(previous);
  ob::State* sa = G[a].state;
  ob::State* sb = G[b].state;

  ob::State* saC1 = C1->allocState();
  ob::State* sbC1 = C1->allocState();
  ExtractC1Subspace(sa, saC1);
  ExtractC1Subspace(sb, sbC1);

  ob::State* saM0 = M0->allocState();
  ob::State* sbM0 = M0->allocState();

  ExtractM0Subspace(sa, saM0);
  ExtractM0Subspace(sb, sbM0);

  const Vertex asM0 = G[a].associated_source;
  const Vertex atM0 = G[a].associated_target;

  const Vertex bsM0 = G[b].associated_source;
  const Vertex btM0 = G[b].associated_target;

  //std::cout << "associated vertices: " << asM0 << "," << atM0 << "|" << bsM0 << "," << btM0 << "." << std::endl;
  ob::PathPtr M0_path = quotient_previous->GetShortestPathOffsetVertices( saM0, sbM0, asM0, bsM0, atM0, btM0);

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

ob::PathPtr QMPConnect::GetShortestPathOffsetVertices(const ob::State *qa, const ob::State *qb, 
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
    const Edge ea = boost::edge(vsa,vta,G).first;
    const Edge eb = boost::edge(vsb,vtb,G).first;
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
    boost::remove_edge(vsa, vta, G);
    va = boost::add_vertex(G);
    G[va].state = si_->cloneState(qa);

    dsa = opt_->motionCost(G[vsa].state, G[va].state);
    dta = opt_->motionCost(G[va].state, G[vta].state);
    boost::add_edge(vsa, va, EdgeInternalState(dsa), G);
    boost::add_edge(va, vta, EdgeInternalState(dta), G);

  }else{
    va = vsa;
    G[va].state = si_->cloneState(qa);
  }

  if(isBonEdge){
    boost::remove_edge(vsb, vtb, G);
    vb = boost::add_vertex(G);
    G[vb].state = si_->cloneState(qb);

    dsb = opt_->motionCost(G[vsb].state, G[vb].state);
    dtb = opt_->motionCost(G[vb].state, G[vtb].state);
    boost::add_edge(vsb, vb, EdgeInternalState(dsb), G);
    boost::add_edge(vb, vtb, EdgeInternalState(dtb), G);
  }else{
    vb = vsb;
    G[vb].state = si_->cloneState(qb);
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
    boost::remove_edge(vsb, vb, G);
    boost::remove_edge(vb, vtb, G);
    si_->freeState(G[vb].state);

    boost::clear_vertex(vb, G);
    boost::remove_vertex(vb, G);

    ob::Cost db = opt_->motionCost(G[vsb].state, G[vtb].state);
    boost::add_edge(vsb, vtb, EdgeInternalState(db), G);
    uniteComponents(vsb, vtb);
  }
  if(isAonEdge){
    boost::remove_edge(vsa, va, G);
    boost::remove_edge(va, vta, G);
    si_->freeState(G[va].state);

    boost::clear_vertex(va, G);
    boost::remove_vertex(va, G);

    ob::Cost da = opt_->motionCost(G[vsa].state, G[vta].state);
    boost::add_edge(vsa, vta, EdgeInternalState(da), G);
    uniteComponents(vsa, vta);
  }

  return path;
}

ompl::PDF<og::QuotientGraph::Edge> QMPConnect::GetEdgePDF()
{
  PDF<Edge> pdf;
  double t = rng_.uniform01();
  if(t<percentageSamplesOnShortestPath)
  {
    //diminishing shortest path sampling
    percentageSamplesOnShortestPath = exp(-pow(((double)samplesOnShortestPath++/1000.0),2));

    foreach (Edge e, boost::edges(G))
    {
      const Vertex v1 = boost::source(e, G);
      const Vertex v2 = boost::target(e, G);
      if(G[v1].on_shortest_path && G[v2].on_shortest_path)
      {
        ob::Cost weight = G[e].getCost();
        //ob::Cost weight = get(boost::edge_weight_t(), G, e).getCost();
        pdf.add(e, weight.value());
      }
    }
  }else{
    //Random Edge (RE) sampling (suffers from high sampling concentrations at
    //vertices with many incoming edges, not ideal, but fast)
    //foreach (Edge e, boost::edges(G))
    //{
    //  const Vertex v1 = boost::source(e, G);

    //  if(sameComponent(v1, startM_.at(0))){
    //    ob::Cost weight = get(boost::edge_weight_t(), Ge).getCost();
    //    pdf.add(e, weight.value());
    //  }
    //}
    ////Random Node Edge (RNE) sampling (better, but still no guarantee of
    //uniformity. Might be good to investigate random walk samplers)
    PDF<Vertex> vpdf;
    foreach (Vertex v, boost::vertices(G))
    {
      if(sameComponent(v, startM_.at(0))){
        vpdf.add(v,1);
      }
    }
    Vertex v = vpdf.sample(rng_.uniform01());
    std::pair<IEIterator, IEIterator> iterators = boost::in_edges(boost::vertex(v, G), G);
    for (IEIterator iter = iterators.first; iter != iterators.second; ++iter)
    {
      //pdf.add(*iter, 1);
      ob::Cost weight = G[*iter].getCost();
      //ob::Cost weight = get(boost::edge_weight_t(), G, *iter).getCost();
      pdf.add(*iter, weight.value());
    }
  }
  return pdf;
}

void QMPConnect::RandomWalk(const Vertex &v) 
{
  const bool DEBUG = false;

  og::QMPConnect *quotient_previous = static_cast<og::QMPConnect*>(previous);
  if(previous == nullptr){
    return QMP::RandomWalk(v);
  }

  Vertex v_first = v;

  uint steps = magic::MAX_RANDOM_BOUNCE_STEPS;
  for (uint i = 0; i < steps; ++i)
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

    if(!M1->isValid(s_last)) continue;

    //#########################################################################
    //compute interpolation between s_prev and s_last on G_{k-1}
    // the resulting datastructures are
    //
    // std::vector<Vertex> vpath
    // std::vector<ob::State *> spath
    //#########################################################################

    Vertex v_last = addMilestone(s_last);

    ob::PathPtr solM1 = InterpolateM1GraphConstraint(v_first, v_last);
    if(!solM1){
      std::cout << "no path towards sampled configuration" << std::endl;
      exit(0);
    }

    og::PathGeometric pathM1 = static_cast<og::PathGeometric&>(*solM1);

    std::vector<ob::State *> spathM1 = pathM1.getStates();
    std::vector<Vertex> vpathM0 = quotient_previous->shortestVertexPath_;

    if(DEBUG){
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "start edge " << G[v_first].associated_source << "<->" 
        << G[v_first].associated_target <<
        " (" << G[v_first].associated_t << ")" 
        << std::endl;

      for(uint k = 1; k < vpathM0.size(); k++){
        std::cout << "edge " << vpathM0.at(k-1) << "<->"<< vpathM0.at(k) << std::endl;
      }

      std::cout << "goal edge " << G[v_last].associated_source << "<->" 
        << G[v_last].associated_target << " (" 
        << G[v_last].associated_t << ")" << std::endl;
    }

    //#########################################################################
    //move along sol/shortestvertexpath_ until infeasible or target reached.
    //#########################################################################

    Vertex v_prevM1 = v_first;

    for(uint k = 1; k < spathM1.size(); k++){

      std::pair<ob::State *, double> lastValid;
      lastValid.first = spathM1.at(k);

      if(DEBUG) std::cout << "edge " << vpathM0.at(k-1) << "<->"<< vpathM0.at(k) << std::endl;

      if(!si_->isValid(spathM1.at(k-1))){
        break;
      }
      if(!si_->checkMotion(spathM1.at(k-1),spathM1.at(k), lastValid)){
        if(lastValid.second < std::numeric_limits<double>::epsilon()){
          break;
        }else{
          Vertex v_nextM1 = CreateNewVertex(lastValid.first);
          if(DEBUG) std::cout << "progress: " <<lastValid.second << std::endl;
          if(k==1){

            if(DEBUG) std::cout << "first" << std::endl;
            Vertex vM0 = G[v_first].associated_source;

            double T1 = G[v_first].associated_t;
            double T2 = lastValid.second;

            if(vM0 == vpathM0.at(1)){
              vM0 = G[v_first].associated_target;
              G[v_nextM1].associated_t=T1-(T2*T1);
            }else{
              G[v_nextM1].associated_t=T1+T2*(1-T1);
            }

            G[v_nextM1].associated_source=vM0;
            G[v_nextM1].associated_target=vpathM0.at(k);


          }else{
            if(k>=spathM1.size()-1){
              //vpathM0(size-2) is the last graph vertex, vpathM0(size-1) is the
              //vertex which we have deleted again.
              if(DEBUG) std::cout << "last" << std::endl;
              Vertex vM0 = G[v_last].associated_target;
              double T1 = G[v_last].associated_t;
              double T2 = lastValid.second;
              if(vM0 == vpathM0.at(vpathM0.size()-2)){
                vM0 = G[v_last].associated_source;
                G[v_nextM1].associated_t=T1+(1-T2)*(1-T1);
              }else{
                G[v_nextM1].associated_t=T1*T2;
              }
              G[v_nextM1].associated_source=vpathM0.at(vpathM0.size()-2);
              G[v_nextM1].associated_target=vM0;

              //double dv = M0->distance(quotient_previous->G[vM0].state, quotient_previous->stateProperty_[vpathM0.at(k)]);
            }else{
              if(DEBUG) std::cout << "in between" << std::endl;

              G[v_nextM1].associated_source=vpathM0.at(k-1);
              G[v_nextM1].associated_target=vpathM0.at(k);
              G[v_nextM1].associated_t=lastValid.second;
            }
          }
          if(DEBUG){
            std::cout << "new vertex associated edges: " 
            << G[v_nextM1].associated_source << "<->"
            << G[v_nextM1].associated_target << " ("
            << G[v_nextM1].associated_t << ")"
            << std::endl;
          }

          double dk = M1->distance(spathM1.at(k), G[v_nextM1].state);
          boost::add_edge(v_prevM1, v_nextM1, EdgeInternalState(ob::Cost(dk)), G);
          uniteComponents(v_prevM1, v_nextM1);
          nn_->add(v_nextM1);
        }
        break;
      }

      if(DEBUG) std::cout << "feasible" << std::endl;
      Vertex v_nextM1;
      if(k<spathM1.size()-1){
        v_nextM1 = CreateNewVertex(spathM1.at(k));
        G[v_nextM1].associated_source=vpathM0.at(k);
        G[v_nextM1].associated_target=vpathM0.at(k);
        G[v_nextM1].associated_t=0;
        nn_->add(v_nextM1);
        totalNumberOfSamples++;
      }else{
        v_nextM1 = v_last;
      }

      double dk = M1->distance(spathM1.at(k-1),spathM1.at(k));
      boost::add_edge(v_prevM1, v_nextM1, EdgeInternalState(ob::Cost(dk)), G);
      uniteComponents(v_prevM1, v_nextM1);
      v_prevM1 = v_nextM1;
    }

  }
}
