#include "quotient_graph.h"

#include "common.h"
#include "planner/strategy/GoalVisitor.hpp"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
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
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace og;

namespace ompl
{
  namespace magic
  {
    static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
    static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
  }
}
QuotientGraph::QuotientGraph(const ob::SpaceInformationPtr &si, Quotient *parent_)
  : BaseT(si, parent_)
{
  setName("QuotientGraph");
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

void QuotientGraph::setup(){
  if (!nn_){
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                             {
                               return Distance(a, b);
                             });
  }
  if (!connectionStrategy_){
    connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
  }

  if (pdef_){
    BaseT::setup();
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      std::cout << "Needs specification of optimization objective." << std::endl;
      exit(0);
    }
    firstRun = true;
    setup_ = true;
  }else{
    setup_ = false;
  }
}

QuotientGraph::~QuotientGraph(){
  si_->freeStates(xstates);
  G.clear();
  if (nn_){
    nn_->clear();
  }
}

void QuotientGraph::ClearVertices()
{
  foreach (Vertex v, boost::vertices(G)){
    si_->freeState(G[v].state);
  }
}
void QuotientGraph::clear()
{
  Quotient::clear();

  ClearVertices();
  G.clear();
  if(nn_){
    nn_->clear();
  }
  clearQuery();
  iterations_ = 0;
  totalNumberOfSamples = 0;
  graphLength = 0;
  bestCost_ = ob::Cost(dInf);
  setup_ = false;
  addedNewSolution_ = false;
  firstRun = true;
}

void QuotientGraph::clearQuery()
{
  startM_.clear();
  goalM_.clear();
  pis_.restart();
}

//@TODO: this were the settings we used for IROS'18. While they worked well in
//our examples, there is no good reason why we used this particular formula.
//Needs revision.
double QuotientGraph::GetImportance() const{
  double N = (double)totalNumberOfSamples;
  //return /((double)si_->getSpaceMeasure());
  return 1.0/(N+1);
    //return N/(parent->GetGraphLength()*X1->getSpaceMeasure());
}

void QuotientGraph::Init()
{
  auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
  if (goal == nullptr){
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    exit(0);
  }

  if(const ob::State *st = pis_.nextStart()){
    if (st != nullptr){
      startM_.push_back(addMilestone(si_->cloneState(st)));
      G[startM_.back()].start = true;
    }
  }
  if (startM_.empty()){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }

  if(const ob::State *st = pis_.nextGoal()){
    if (st != nullptr){
      goalM_.push_back(addMilestone(si_->cloneState(st)));
      G[goalM_.back()].goal = true;
    }
  }
  if (goalM_.empty()){
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
  //unsigned long int nrStartStates = boost::num_vertices(G);
  //OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
}

void QuotientGraph::Grow(double t){
  if(firstRun){
    Init();
    firstRun = false;
  }
  double T_grow = (2.0/3.0)*t;
  growRoadmap(ob::timedPlannerTerminationCondition(T_grow), xstates[0]);
  double T_expand = (1.0/3.0)*t;
  expandRoadmap( ob::timedPlannerTerminationCondition(T_expand), xstates);
}

void QuotientGraph::growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState)
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

void QuotientGraph::expandRoadmap(const ob::PlannerTerminationCondition &ptc,
                                         std::vector<ob::State *> &workStates)
{
  PDF<Vertex> pdf;
  //find all nodes which have been tried to expand often (frontier nodes), but
  //which have not been successfully expanded (boundary nodes). In that case the
  //vertex has a large voronoi bias but has been stuck. The proposed solution
  //here starts at the vertex, and does a random walk (randombouncemotion), to
  //try to get unstuck.

  foreach (Vertex v, boost::vertices(G))
  {
    const unsigned long int t = G[v].total_connection_attempts;
    if(t!=0){
      double d = ((double)(t - G[v].successful_connection_attempts) / (double)t);
      pdf.add(v, d);
    }
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

void QuotientGraph::uniteComponents(Vertex m1, Vertex m2)
{
  disjointSets_.union_set(m1, m2);
}

bool QuotientGraph::sameComponent(Vertex m1, Vertex m2)
{
  return boost::same_component(m1, m2, disjointSets_);
}

QuotientGraph::Vertex QuotientGraph::addMilestone(ob::State *state)
{
  Vertex m = CreateNewVertex(state);
  ConnectVertexToNeighbors(m);
  return m;
}

void QuotientGraph::ConnectVertexToNeighbors(Vertex m)
{
  const std::vector<Vertex> &neighbors = connectionStrategy_(m);

  foreach (Vertex n, neighbors)
  {
    G[m].total_connection_attempts++;
    G[n].total_connection_attempts++;
    if(Connect(m,n)){
      G[m].successful_connection_attempts++;
      G[n].successful_connection_attempts++;
    }
  }
  nn_->add(m);
}

const og::QuotientGraph::Graph& QuotientGraph::GetGraph() const
{
  return G;
}
const og::QuotientGraph::RoadmapNeighborsPtr& QuotientGraph::GetRoadmapNeighborsPtr() const
{
  return nn_;
}
const og::QuotientGraph::ConnectionStrategy& QuotientGraph::GetConnectionStrategy() const
{
  return connectionStrategy_;
}

QuotientGraph::Vertex QuotientGraph::CreateNewVertex(ob::State *state)
{
  Vertex m = boost::add_vertex(G);
  G[m].state = si_->cloneState(state);
  G[m].total_connection_attempts = 1;
  G[m].successful_connection_attempts = 0;
  disjointSets_.make_set(m);
  return m;
}


ob::Cost QuotientGraph::costHeuristic(Vertex u, Vertex v) const
{
  return opt_->motionCostHeuristic(G[u].state, G[v].state);
}

uint QuotientGraph::GetNumberOfVertices() const{
  return num_vertices(G);
}

uint QuotientGraph::GetNumberOfEdges() const{
  return num_edges(G);
}

template <template <typename T> class NN>
void QuotientGraph::setNearestNeighbors()
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

double QuotientGraph::Distance(const Vertex a, const Vertex b) const
{
  return si_->distance(G[a].state, G[b].state);
}

bool QuotientGraph::Connect(const Vertex a, const Vertex b){
  if (si_->checkMotion(G[a].state, G[b].state))
  {
    ob::Cost weight = opt_->motionCost(G[a].state, G[b].state);
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, G);
    uniteComponents(a, b);
    return true;
  }
  return false;
}

void QuotientGraph::RandomWalk(const Vertex &v)
{
  const ob::State *s_prev = G[v].state;

  Vertex v_prev = v;

  uint ctr = 0;
  for (uint i = 0; i < magic::MAX_RANDOM_BOUNCE_STEPS; ++i)
  {
    ob::State *s_next = xstates[ctr];
    Q1_sampler->sampleUniform(s_next);

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

      EdgeInternalState properties(opt_->motionCost(G[v_prev].state, G[v_next].state));
      boost::add_edge(v_prev, v_next, properties, G);
      uniteComponents(v_prev, v_next);
      nn_->add(v_next);

      v_prev = v_next;
      s_prev = G[v_next].state;
      ctr++;
    }
  }
}

double QuotientGraph::GetGraphLength() const{
  return graphLength;
}

bool QuotientGraph::GetSolution(ob::PathPtr &solution)
{
  if(hasSolution){
    solution_path = GetPath(startM_.back(), goalM_.back());
    startGoalVertexPath_ = shortestVertexPath_;
    solution = solution_path;
    return true;
  }else{
    ob::Goal *g = pdef_->getGoal().get();
    bestCost_ = ob::Cost(+dInf);
    foreach (Vertex start, startM_)
    {
      foreach (Vertex goal, goalM_)
      {
        bool same_component = sameComponent(start, goal);

        if (same_component && g->isStartGoalPairValid(G[goal].state, G[start].state))
        {
          solution_path = GetPath(start, goal);
          if (solution_path)
          {
            solution = solution_path;
            hasSolution = true;
            startGoalVertexPath_ = shortestVertexPath_;
            return true;
          }
        }
      }
    }
  }
  return hasSolution;
}
ob::PathPtr QuotientGraph::GetPath(const Vertex &start, const Vertex &goal)
{
  std::vector<Vertex> prev(boost::num_vertices(G));
  auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost), get(boost::edge_bundle, G));
  try
  {
    boost::astar_search(G, start,
                      [this, goal](const Vertex v)
                      {
                          return costHeuristic(v, goal);
                      },
                      boost::predecessor_map(&prev[0])
                        .weight_map(weight)
                        .distance_compare([this](EdgeInternalState c1, EdgeInternalState c2)
                                          {
                                              return opt_->isCostBetterThan(c1.getCost(), c2.getCost());
                                          })
                        .distance_combine([this](EdgeInternalState c1, EdgeInternalState c2)
                                          {
                                              return opt_->combineCosts(c1.getCost(), c2.getCost());
                                          })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                      );
  }
  catch (AStarFoundGoal &)
  {
  }

  auto p(std::make_shared<PathGeometric>(si_));
  if (prev[goal] == goal){
    return nullptr;
  }

  std::vector<Vertex> vpath;
  for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos]){
    G[pos].on_shortest_path = true;
    vpath.push_back(pos);
    p->append(G[pos].state);
  }
  G[start].on_shortest_path = true;
  vpath.push_back(start);
  p->append(G[start].state);

  shortestVertexPath_.clear();
  shortestVertexPath_.insert( shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend() );
  p->reverse();

  return p;
}
bool QuotientGraph::InsideStartComponent(Vertex v)
{
  return sameComponent(v, startM_.at(0));
}
bool QuotientGraph::InsideStartComponent(Edge e)
{
  return sameComponent(startM_.at(0), boost::source(e,G));
}

void QuotientGraph::getPlannerData(ob::PlannerData &data) const
{
  //in case we havent found a solution, there will be at 
  //least two connected components which we like to color differently

  uint startComponent = 0;//const_cast<QuotientGraph *>(this)->disjointSets_.find_set(startM_.at(0));
  uint goalComponent = 1;//const_cast<QuotientGraph *>(this)->disjointSets_.find_set(goalM_.at(0));
  for (unsigned long i : startM_)
  {
    startComponent = const_cast<QuotientGraph *>(this)->disjointSets_.find_set(i);
    PlannerDataVertexAnnotated pstart(G[i].state, startComponent);
    pstart.SetComponent(0);
    data.addStartVertex(pstart);

  }

  for (unsigned long i : goalM_)
  {
    PlannerDataVertexAnnotated pgoal(G[i].state, const_cast<QuotientGraph *>(this)->disjointSets_.find_set(i));
    pgoal.SetComponent(1);
    data.addGoalVertex(pgoal);
  }

  std::cout << "vertices " << GetNumberOfVertices() << " edges " << GetNumberOfEdges() << std::endl;
  uint ctr = 0;
  foreach (const Edge e, boost::edges(G))
  {
    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);

    PlannerDataVertexAnnotated p1(G[v1].state);
    PlannerDataVertexAnnotated p2(G[v2].state);

    uint vi1 = data.addVertex(p1);
    uint vi2 = data.addVertex(p2);

    data.addEdge(p1,p2);
    ctr++;

    uint v1Component = const_cast<QuotientGraph *>(this)->disjointSets_.find_set(v1);
    uint v2Component = const_cast<QuotientGraph *>(this)->disjointSets_.find_set(v2);
    PlannerDataVertexAnnotated &v1a = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vi1));
    PlannerDataVertexAnnotated &v2a = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vi2));
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
}
bool QuotientGraph::SampleQuotient(ob::State *q_random_graph)
{
  if(num_edges(G) == 0) return false;

  Edge e = boost::random_edge(G, rng_boost);
  while(!sameComponent(boost::source(e, G), startM_.at(0)))
  {
    e = boost::random_edge(G, rng_boost);
  }

  double s = rng_.uniform01();

  const Vertex v1 = boost::source(e, G);
  const Vertex v2 = boost::target(e, G);
  const ob::State *from = G[v1].state;
  const ob::State *to = G[v2].state;

  Q1->getStateSpace()->interpolate(from, to, s, q_random_graph);
  return true;
}
void QuotientGraph::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << "[QuotientGraph has " << GetNumberOfVertices() << " vertices and " << GetNumberOfEdges() << " edges.]" << std::endl;
}

