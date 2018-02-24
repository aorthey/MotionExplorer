#include "prm_basic.h"
#include "common.h"
#include "GoalVisitor.hpp"
#include "planner/validitychecker/validity_checker_ompl.h"
#include "elements/plannerdata_vertex_annotated.h"

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace og;

namespace ompl
{
  namespace magic
  {
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 3;
    static const double ROADMAP_BUILD_TIME = 0.01;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 5;
  }
}
PRMBasic::PRMBasic(const ob::SpaceInformationPtr &si, Quotient *previous_)
  : Quotient(si, previous_)
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_))
  , associatedVertexSourceProperty_(boost::get(vertex_associated_vertex_source_t(), g_))
  , associatedVertexTargetProperty_(boost::get(vertex_associated_vertex_target_t(), g_))
  , associatedTProperty_(boost::get(vertex_associated_t_t(), g_))
  , onShortestPath_(boost::get(vertex_on_shortest_path_t(), g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
{
  setName("PRMBasic");
  specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = false;

  if (!isSetup())
  {
    setup();
  }

  xstates.resize(magic::MAX_RANDOM_BOUNCE_STEPS);
  si_->allocStates(xstates);
  totalNumberOfSamples = 0;
}

PRMBasic::~PRMBasic(){
  si_->freeStates(xstates);
  g_.clear();
  if (nn_){
    nn_->clear();
  }
}

void PRMBasic::ClearVertices()
{
  foreach (Vertex v, boost::vertices(g_)){
    si_->freeState(stateProperty_[v]);
  }
}
void PRMBasic::clear()
{
  Planner::clear();

  ClearVertices();
  g_.clear();
  if (nn_){
    nn_->clear();
  }
  clearQuery();

  iterations_ = 0;
  bestCost_ = ob::Cost(dInf);
}

void PRMBasic::clearQuery()
{
  startM_.clear();
  goalM_.clear();
  pis_.restart();
}

void PRMBasic::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
  Planner::setProblemDefinition(pdef);
}

void PRMBasic::Init(){
  checkValidity();
}

ob::PlannerStatus PRMBasic::solve(const ob::PlannerTerminationCondition &ptc){
  Init();

  addedNewSolution_ = false;
  base::PathPtr sol;

  bestCost_ = opt_->infiniteCost();

  base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                 { return ptc || addedNewSolution_; });

  while (!ptcOrSolutionFound())
  {
    Grow(magic::ROADMAP_BUILD_TIME);
    CheckForSolution(sol);
  }

  OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_));

  if (sol)
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    psol.setOptimized(opt_, bestCost_, addedNewSolution_);
    pdef_->addSolutionPath(psol);
  }

  return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void PRMBasic::Grow(double t){
  double T_grow = (2.0/3.0)*t;
  growRoadmap(ob::timedPlannerTerminationCondition(T_grow), xstates[0]);
  //double T_expand = (1.0/3.0)*t;
  //expandRoadmap( ob::timedPlannerTerminationCondition(T_expand), xstates);
}

void PRMBasic::growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState)
{
  while (!ptc)
  {
    iterations_++;
    bool found = false;
    while (!found && !ptc)
    {
      unsigned int attempts = 0;
      do
      {
        found = Sample(workState);
        attempts++;
      } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
    }
    if (found) addMilestone(si_->cloneState(workState));
  }
}

void PRMBasic::expandRoadmap(const ob::PlannerTerminationCondition &ptc,
                                         std::vector<ob::State *> &workStates)
{
  PDF<Vertex> pdf;
  //find all nodes which have been tried to expand often (frontier nodes), but
  //which have not been successfully expanded (boundary nodes). In that case the
  //vertex has a large voronoi bias but has been stuck. The proposed solution
  //here starts at the vertex, and does a random walk (randombouncemotion), to
  //try to get unstuck.

  foreach (Vertex v, boost::vertices(g_))
  {
    const unsigned long int t = totalConnectionAttemptsProperty_[v];
    if(t!=0){
      double d = ((double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
      pdf.add(v, d);
    }
    // if(std::isnan(d)){
    //   std::cout << "vertex " << v << " d=" << d << std::endl;
    //   std::cout << "total   =" << totalConnectionAttemptsProperty_[v] << std::endl;
    //   std::cout << "success =" << successfulConnectionAttemptsProperty_[v] << std::endl;
    //   exit(0);
    // }
  }

  if (pdf.empty())
    return;

  while (!ptc)
  {
    iterations_++;
    Vertex v = pdf.sample(rng_.uniform01());
    RandomWalk(v);
  }
}
void PRMBasic::uniteComponents(Vertex m1, Vertex m2)
{
  disjointSets_.union_set(m1, m2);
}

bool PRMBasic::sameComponent(Vertex m1, Vertex m2)
{
  return boost::same_component(m1, m2, disjointSets_);
}

void PRMBasic::CheckForSolution(ob::PathPtr &solution)
{
  hasSolution = maybeConstructSolution(startM_, goalM_, solution);
}

bool PRMBasic::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                                  ob::PathPtr &solution)
{
  ob::Goal *g = pdef_->getGoal().get();
  bestCost_ = ob::Cost(+dInf);
  foreach (Vertex start, starts)
  {
    foreach (Vertex goal, goals)
    {
      bool same_component = sameComponent(start, goal);

      if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
      {
        ob::PathPtr p = constructSolution(start, goal);
        if (p)
        {
          ob::Cost pathCost = p->cost(opt_);

          if (opt_->isCostBetterThan(pathCost, bestCost_)){
            bestCost_ = pathCost;
          }
          if (opt_->isSatisfied(pathCost))
          {
            solution = p;
            //#################################################################
            //clean up roadmap
            //#################################################################
            // std::vector<Edge> unconnectedEdges;
            // uint ctr = 0;
            // foreach (Edge e, boost::edges(g_))
            // {
            //   ctr++;
            //   const Vertex v1 = boost::source(e, g_);
            //   if(!sameComponent(v1, startM_.at(0))){
            //     unconnectedEdges.push_back(e);
            //   }
            // }
            // std::cout << "found " << unconnectedEdges.size() << "/" << ctr <<" unconnectedEdges"  << std::endl;
            // for(uint k = 0; k < unconnectedEdges.size(); k++){
            //   Edge e = unconnectedEdges.at(k);
            //   const Vertex v1 = boost::source(e, g_);
            //   const Vertex v2 = boost::target(e, g_);
            //   boost::remove_edge(boost::vertex(v1, g_), boost::vertex(v2,g_), g_);
            // }
            // //uint Nv = boost::num_vertices(g_);
            // ctr = 0;
            // foreach (Vertex v, boost::vertices(g_))
            // {
            //   int numberOfInEdges = boost::in_degree(v,g_);
            //   if(v>1 && numberOfInEdges<=0)
            //   {
            //     ctr++;
            //     //si_->freeState(stateProperty_[v]);
            //     nn_->remove(v);
            //     //boost::remove_vertex(boost::vertex(v, g_), g_);
            //   }
            // }
            // uniteComponents(startM_.at(0), goalM_.at(0));
            //std::cout << "removed " << ctr << "/" << Nv <<" vertices."  << std::endl;
            //#################################################################
            //#################################################################
            return true;
          }
        }
      }
    }
  }

  return false;
}
ob::PathPtr PRMBasic::constructSolution(const Vertex &start, const Vertex &goal)
{
  boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

  try
  {
      boost::astar_search(g_, start,
                          [this, goal](Vertex v)
                          {
                              return costHeuristic(v, goal);
                          },
                          boost::predecessor_map(prev)
                              .distance_compare([this](EdgeProperty c1, EdgeProperty c2)
                                                {
                                                    return opt_->isCostBetterThan(c1.getCost(), c2.getCost());
                                                })
                              .distance_combine([this](EdgeProperty c1, EdgeProperty c2)
                                                {
                                                    return opt_->combineCosts(c1.getCost(), c2.getCost());
                                                })
                              .distance_inf(opt_->infiniteCost())
                              .distance_zero(opt_->identityCost())
                              .visitor(AStarGoalVisitor<Vertex>(goal)));
  }
  catch (AStarFoundGoal &)
  {
  }

  auto p(std::make_shared<PathGeometric>(si_));
  if (prev[goal] == goal){
    return NULL;
  }

  std::vector<Vertex> vpath;
  for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos]){
    onShortestPath_[pos] = true;
    vpath.push_back(pos);
    p->append(stateProperty_[pos]);
  }
  onShortestPath_[start] = true;
  vpath.push_back(start);
  p->append(stateProperty_[start]);

  shortestVertexPath_.clear();
  shortestVertexPath_.insert( shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend() );
  p->reverse();

  return p;
}


PRMBasic::Vertex PRMBasic::addMilestone(ob::State *state)
{
  Vertex m = CreateNewVertex(state);
  ConnectVertexToNeighbors(m);
  return m;
}

void PRMBasic::ConnectVertexToNeighbors(Vertex m)
{
  const std::vector<Vertex> &neighbors = connectionStrategy_(m);

  foreach (Vertex n, neighbors)
  {
    totalConnectionAttemptsProperty_[m]++;
    totalConnectionAttemptsProperty_[n]++;
    if(Connect(m,n)){
      graphLength += Distance(m, n);
      successfulConnectionAttemptsProperty_[m]++;
      successfulConnectionAttemptsProperty_[n]++;
    }
  }
  nn_->add(m);
}

PRMBasic::Vertex PRMBasic::CreateNewVertex(ob::State *state)
{
  Vertex m = boost::add_vertex(g_);
  stateProperty_[m] = si_->cloneState(state);
  totalConnectionAttemptsProperty_[m] = 0;
  successfulConnectionAttemptsProperty_[m] = 0;
  onShortestPath_[m] = false;
  disjointSets_.make_set(m);
  return m;
}

void PRMBasic::setup(){
  if (!nn_){
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                             {
                               return Distance(a, b);
                             });
  }
  if (!connectionStrategy_){
    //connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
    connectionStrategy_ = KStarStrategy<Vertex>(
    [this]
    {
        return GetNumberOfVertices();
    },
    nn_, si_->getStateDimension());
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

    while (const ob::State *st = pis_.nextStart()){
      startM_.push_back(addMilestone(si_->cloneState(st)));
    }
    if (startM_.empty()){
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      //const ob::State *start = pdef_->getStartState(0);
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
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
  }else{
    //OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
    setup_ = false;
  }

}

ob::Cost PRMBasic::costHeuristic(Vertex u, Vertex v) const
{
  return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

ob::PathPtr PRMBasic::GetShortestPath(){
  return GetSolutionPath();
}

uint PRMBasic::GetNumberOfVertices(){
  return num_vertices(g_);
}

uint PRMBasic::GetNumberOfEdges(){
  return num_edges(g_);
}

ob::PathPtr PRMBasic::GetSolutionPath(){
  ob::PathPtr sol;
  CheckForSolution(sol);
  return sol;
}

template <template <typename T> class NN>
void PRMBasic::setNearestNeighbors()
{
  if (nn_ && nn_->size() == 0)
      OMPL_WARN("Calling setNearestNeighbors will clear all states.");
  clear();
  nn_ = std::make_shared<NN<Vertex>>();
  connectionStrategy_ = ConnectionStrategy();
  if(!isSetup()){
    setup();
  }
}

double PRMBasic::Distance(const Vertex a, const Vertex b) const
{
  return si_->distance(stateProperty_[a], stateProperty_[b]);
}

bool PRMBasic::Connect(const Vertex a, const Vertex b){
  if (si_->checkMotion(stateProperty_[a], stateProperty_[b]))
  {
    ob::Cost weight = opt_->motionCost(stateProperty_[a], stateProperty_[b]);
    EdgeProperty properties(weight);
    boost::add_edge(a, b, properties, g_);
    uniteComponents(a, b);
    return true;
  }
  return false;
}

//uint PRMBasic::randomBounceMotion(const Vertex &v, std::vector<ob::State *> &states) const
//{
//  uint steps = states.size();
//  const ob::State *prev = stateProperty_[v];
//  std::pair<ob::State *, double> lastValid;
//  uint j = 0;
//  for (uint i = 0; i < steps; ++i)
//  {
//    M1_sampler->sampleUniform(states[j]);
//    lastValid.first = states[j];
//    if (si_->checkMotion(prev, states[j], lastValid) || lastValid.second > std::numeric_limits<double>::epsilon())
//      prev = states[j++];
//  }
//  return j;
//}

void PRMBasic::RandomWalk(const Vertex &v)
{
  const ob::State *s_prev = stateProperty_[v];

  Vertex v_prev = v;

  uint ctr = 0;
  for (uint i = 0; i < magic::MAX_RANDOM_BOUNCE_STEPS; ++i)
  {
    //s_next = SAMPLE(M1)

    ob::State *s_next = xstates[ctr];
    M1_sampler->sampleUniform(s_next);

    std::pair<ob::State *, double> lastValid;
    lastValid.first = s_next;

    //check if motion is valid: s_prev -------- s_next
    //s_prev ------ lastValid ----------------s_next
    if(!si_->isValid(s_prev)){
      continue;
    }
    si_->checkMotion(s_prev, s_next, lastValid);

    //if we made progress towards s_next, then add the last valid state to our
    //roadmap, connect it to the s_prev state
    if(lastValid.second > std::numeric_limits<double>::epsilon())
    {
      Vertex v_next = CreateNewVertex(lastValid.first);

      EdgeProperty properties(opt_->motionCost(stateProperty_[v_prev], stateProperty_[v_next]));
      boost::add_edge(v_prev, v_next, properties, g_);
      uniteComponents(v_prev, v_next);
      nn_->add(v_next);

      v_prev = v_next;
      s_prev = stateProperty_[v_next];
      ctr++;
    }
  }
}

void PRMBasic::getPlannerData(ob::PlannerData &data) const
{
  for (unsigned long i : startM_)
    data.addStartVertex(
      PlannerDataVertexAnnotated(stateProperty_[i], const_cast<PRMBasic *>(this)->disjointSets_.find_set(i)));

  for (unsigned long i : goalM_)
    data.addGoalVertex(
      PlannerDataVertexAnnotated(stateProperty_[i], const_cast<PRMBasic *>(this)->disjointSets_.find_set(i)));

  std::cout << "  edges    : " << boost::num_edges(g_) << std::endl;
  std::cout << "  vertices : " << boost::num_vertices(g_) << std::endl;
  uint startComponent = const_cast<PRMBasic *>(this)->disjointSets_.find_set(startM_.at(0));
  uint goalComponent = const_cast<PRMBasic *>(this)->disjointSets_.find_set(goalM_.at(0));
  foreach (const Edge e, boost::edges(g_))
  {
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    PlannerDataVertexAnnotated p1(stateProperty_[v1]);
    PlannerDataVertexAnnotated p2(stateProperty_[v2]);

    //Vertex d1 = data.addVertex(p1);
    //Vertex d2 = data.addVertex(p2);
    data.addEdge(p1,p2);

    //data.tagState(stateProperty_[v1], const_cast<PRMBasic *>(this)->disjointSets_.find_set(v1));
    //data.tagState(stateProperty_[v2], const_cast<PRMBasic *>(this)->disjointSets_.find_set(v2));
    uint v1Component = const_cast<PRMBasic *>(this)->disjointSets_.find_set(v1);
    uint v2Component = const_cast<PRMBasic *>(this)->disjointSets_.find_set(v2);
    PlannerDataVertexAnnotated &v1a = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(v1));
    PlannerDataVertexAnnotated &v2a = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(v2));
    if(v1Component==startComponent || v2Component==startComponent){
      v1a.SetComponent(0);
      v2a.SetComponent(0);
    }else if(v1Component==goalComponent || v2Component==goalComponent){
      v1a.SetComponent(1);
      v2a.SetComponent(1);
    }else{
      v1a.SetComponent(2);
      v2a.SetComponent(2);
    }
  }

  // //foreach(const Vertex v, boost::vertices(g_))
  // for(long unsigned int v = 0; v < data.numVertices(); v++)
  // {
  //   PlannerDataVertexAnnotated &va = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(v));
  //   if(boost::same_component(v, startM_.at(0), const_cast<PRMBasic*>(this)->disjointSets_))
  //   {
  //     va.SetComponent(0);
  //   }else{
  //     if(boost::same_component(v, goalM_.at(0), const_cast<PRMBasic*>(this)->disjointSets_))
  //     {
  //       va.SetComponent(1);
  //     }else{
  //       va.SetComponent(2);
  //     }
  //   }
  // }
  // std::cout << std::string(80, '-') << std::endl;
}

