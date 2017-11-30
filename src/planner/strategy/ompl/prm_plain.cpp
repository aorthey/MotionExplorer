#include "prm_plain.h"
#include "GoalVisitor.hpp"

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
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
    static const double ROADMAP_BUILD_TIME = 0.01;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}
PRMPlain::PRMPlain(const ob::SpaceInformationPtr &si)
  : ob::Planner(si, "PRMPlain")
  , stateProperty_(boost::get(vertex_state_t(), graph))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), graph))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), graph))
  , disjointSets_(boost::get(boost::vertex_rank, graph), boost::get(boost::vertex_predecessor, graph))
{
  specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;

  if (!isSetup())
    setup();
  if (!sampler_){
    sampler_ = si_->allocValidStateSampler();
  }
  if (!simpleSampler_){
    simpleSampler_ = si_->allocStateSampler();
  }

  xstates.resize(magic::MAX_RANDOM_BOUNCE_STEPS);
  si_->allocStates(xstates);

  std::cout << "Hello this is PRMPlain with measure=" << si_->getSpaceMeasure() << std::endl;
}

PRMPlain::~PRMPlain(){
  si_->freeStates(xstates);
}

void PRMPlain::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
  Planner::setProblemDefinition(pdef);
}
ob::PlannerStatus PRMPlain::Init(const base::PlannerTerminationCondition &ptc){
  checkValidity();
  unsigned long int nrStartStates = boost::num_vertices(graph);
  OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);
}

ob::PlannerStatus PRMPlain::solve(const ob::PlannerTerminationCondition &ptc){
  Init(ptc);

  addedNewSolution_ = false;
  base::PathPtr sol;

  bestCost_ = opt_->infiniteCost();

  base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                 { return ptc || addedNewSolution_; });

  while (!ptcOrSolutionFound())
  {
    Grow(magic::ROADMAP_BUILD_TIME);
    checkForSolution(sol);
  }

  OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(graph));

  if (sol)
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    psol.setOptimized(opt_, bestCost_, addedNewSolution_);
    pdef_->addSolutionPath(psol);
  }

  return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void PRMPlain::Grow(double t){
  double twothird = (2.0/3.0)*t;
  double onethird = (1.0/3.0)*t;
  growRoadmap(ob::timedPlannerTerminationCondition(twothird), xstates[0]);
  expandRoadmap( ob::timedPlannerTerminationCondition(onethird), xstates);
}

void PRMPlain::growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState)
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
        found = sampler_->sample(workState);
        attempts++;
      } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
    }
    if (found) addMilestone(si_->cloneState(workState));
  }
}

void PRMPlain::expandRoadmap(const ob::PlannerTerminationCondition &ptc,
                                         std::vector<ob::State *> &workStates)
{
    PDF<Vertex> pdf;
    foreach (Vertex v, boost::vertices(graph))
    {
      const unsigned long int t = totalConnectionAttemptsProperty_[v];
      pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
    }

    if (pdf.empty())
      return;

    while (!ptc)
    {
      iterations_++;
      Vertex v = pdf.sample(rng_.uniform01());
      unsigned int s =
          si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
      if (s > 0)
      {
          s--;
          Vertex last = addMilestone(si_->cloneState(workStates[s]));

          for (unsigned int i = 0; i < s; ++i)
          {
              // add the vertex along the bouncing motion
              Vertex m = boost::add_vertex(graph);
              stateProperty_[m] = si_->cloneState(workStates[i]);
              totalConnectionAttemptsProperty_[m] = 1;
              successfulConnectionAttemptsProperty_[m] = 0;
              disjointSets_.make_set(m);

              // add the edge to the parent vertex
              //const Graph::edge_property_type properties(weight);
              EdgeProperty properties(opt_->motionCost(stateProperty_[v], stateProperty_[m]));
              boost::add_edge(v, m, properties, graph);
              uniteComponents(v, m);

              // add the vertex to the nearest neighbors data structure
              nn_->add(m);
              v = m;
          }

          // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
          // we add an edge
          if (s > 0 || !sameComponent(v, last))
          {
            // add the edge to the parent vertex
            //const Graph::edge_property_type properties(weight);
            EdgeProperty properties(opt_->motionCost(stateProperty_[v], stateProperty_[last]));
            boost::add_edge(v, last, properties, graph);
            uniteComponents(v, last);
          }
      }
    }
}
void PRMPlain::uniteComponents(Vertex m1, Vertex m2)
{
  disjointSets_.union_set(m1, m2);
}

bool PRMPlain::sameComponent(Vertex m1, Vertex m2)
{
  return boost::same_component(m1, m2, disjointSets_);
}

void PRMPlain::checkForSolution(ob::PathPtr &solution)
{
  bool foundSolution = maybeConstructSolution(startM_, goalM_, solution);
  if(foundSolution && !addedNewSolution_){
    addedNewSolution_ = true;
  }
}

bool PRMPlain::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                                  ob::PathPtr &solution)
{
  ob::Goal *g = pdef_->getGoal().get();
  ob::Cost sol_cost(opt_->infiniteCost());
  bestCost_ = ob::Cost(+dInf);
  foreach (Vertex start, starts)
  {
    foreach (Vertex goal, goals)
    {
      // we lock because the connected components algorithm is incremental and may change disjointSets_
      bool same_component = sameComponent(start, goal);

      if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
      {
        ob::PathPtr p = constructSolution(start, goal);
        if (p)
        {
          ob::Cost pathCost = p->cost(opt_);
          //bool better = opt_->isCostBetterThan(pathCost, bestCost_);
          //std::cout << pathCost.value() << (better?">":"<") << bestCost_.value() << std::endl;

          if (opt_->isCostBetterThan(pathCost, bestCost_)){
            bestCost_ = pathCost;
          }
          if (opt_->isSatisfied(pathCost))
          {
            solution = p;
            return true;
          }
          if (opt_->isCostBetterThan(pathCost, sol_cost))
          {
            solution = p;
            sol_cost = pathCost;
          }
        }
      }
    }
  }

  return false;
}
ob::PathPtr PRMPlain::constructSolution(const Vertex &start, const Vertex &goal)
{
    boost::vector_property_map<Vertex> prev(boost::num_vertices(graph));

    try
    {
        boost::astar_search(graph, start,
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

    last_vertex_path.clear();
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos]){
      last_vertex_path.push_back(pos);
      p->append(stateProperty_[pos]);
    }
    last_vertex_path.push_back(start);
    p->append(stateProperty_[start]);
    p->reverse();

    std::reverse(std::begin(last_vertex_path), std::end(last_vertex_path));

    return p;
}
PRMPlain::Vertex PRMPlain::addMilestone(ob::State *state)
{
  Vertex m = boost::add_vertex(graph);
  stateProperty_[m] = state;
  totalConnectionAttemptsProperty_[m] = 1;
  successfulConnectionAttemptsProperty_[m] = 0;

  disjointSets_.make_set(m);
  const std::vector<Vertex> &neighbors = connectionStrategy_(m);

  foreach (Vertex n, neighbors)
  {
    if (connectionFilter_(n, m))
    {
      totalConnectionAttemptsProperty_[m]++;
      totalConnectionAttemptsProperty_[n]++;
      if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
      {
        successfulConnectionAttemptsProperty_[m]++;
        successfulConnectionAttemptsProperty_[n]++;
        EdgeProperty properties(opt_->motionCost(stateProperty_[n], stateProperty_[m]));
        boost::add_edge(n, m, properties, graph);
        uniteComponents(n, m);
      }
    }
  }

  nn_->add(m);

  return m;
}


void PRMPlain::getPlannerData(ob::PlannerData &data) const
{
    for (unsigned long i : startM_)
        data.addStartVertex(
            ob::PlannerDataVertex(stateProperty_[i], const_cast<PRMPlain *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : goalM_)
        data.addGoalVertex(
            ob::PlannerDataVertex(stateProperty_[i], const_cast<PRMPlain *>(this)->disjointSets_.find_set(i)));

    std::cout << "  edges : " << boost::num_edges(graph) << std::endl;
    foreach (const Edge e, boost::edges(graph))
    {
        const Vertex v1 = boost::source(e, graph);
        const Vertex v2 = boost::target(e, graph);
        data.addEdge(ob::PlannerDataVertex(stateProperty_[v1]), ob::PlannerDataVertex(stateProperty_[v2]));
        data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]), ob::PlannerDataVertex(stateProperty_[v1]));
        data.tagState(stateProperty_[v1], const_cast<PRMPlain *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2], const_cast<PRMPlain *>(this)->disjointSets_.find_set(v2));
    }
}
void PRMPlain::setup(){
  if (!nn_){
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                             {
                                 return distanceFunction(a, b);
                             });
  }
  if (!connectionStrategy_){
    connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
  }

  if (!connectionFilter_){
    connectionFilter_ = [](const Vertex &, const Vertex &)
    {
      return true;
    };
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
    unsigned long int nrStartStates = boost::num_vertices(graph);
    OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
  }else{
    //OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
    setup_ = false;
  }

}
void PRMPlain::clear()
{
  foreach (Vertex v, boost::vertices(graph)){
    si_->freeState(stateProperty_[v]);
  }
  graph.clear();
  sampler_.reset();
  simpleSampler_.reset();
  if (nn_)
      nn_->clear();
  clearQuery();

  iterations_ = 0;
  bestCost_ = ob::Cost(dInf);
}

ob::Cost PRMPlain::costHeuristic(Vertex u, Vertex v) const
{
  return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

double PRMPlain::distanceFunction(const Vertex a, const Vertex b) const
{
  return si_->distance(stateProperty_[a], stateProperty_[b]);
}

void PRMPlain::clearQuery()
{
  startM_.clear();
  goalM_.clear();
  pis_.restart();
}

double PRMPlain::GetSamplingDensity(){
  return (double)num_vertices(graph)/(double)si_->getSpaceMeasure();
}

ob::PathPtr PRMPlain::GetShortestPath(){
  return GetSolutionPath();
}

ob::PathPtr PRMPlain::GetSolutionPath(){
  ob::PathPtr sol;
  checkForSolution(sol);
  return sol;
}

bool PRMPlain::hasSolution(){
  if(bestCost_.value() < dInf){
    return addedNewSolution_;
  }else{
    return false;
  }
}

template <template <typename T> class NN>
void PRMPlain::setNearestNeighbors()
{
    if (nn_ && nn_->size() == 0)
        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nn_ = std::make_shared<NN<Vertex>>();
    connectionStrategy_ = ConnectionStrategy();
    if (isSetup())
        setup();
}

void PRMPlain::setConnectionFilter(const ConnectionFilter &connectionFilter)
{
    connectionFilter_ = connectionFilter;
}
