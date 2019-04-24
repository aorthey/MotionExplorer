#include "common.h"
#include "quotient_chart_subgraph.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

QuotientChartSubGraph::QuotientChartSubGraph(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QuotientChartSubGraph"+std::to_string(id));
  q_random = new Configuration(Q1);
}

QuotientChartSubGraph::~QuotientChartSubGraph(void)
{
}
//#############################################################################
//SETUP
//#############################################################################

double QuotientChartSubGraph::Distance(const Configuration* a, const Configuration* b) const
{
  return Q1->distance(a->state, b->state);
}

void QuotientChartSubGraph::setup(void)
{
  BaseT::setup();
  goal = pdef_->getGoal().get();
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);

  if (!nearest_configuration){
    nearest_configuration.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
    nearest_configuration->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                                return Distance(a,b);
                              });
  }

  isConnected = false;

  if (pdef_){
    //#########################################################################
    if (pdef_->hasOptimizationObjective()){
      opt_ = pdef_->getOptimizationObjective();
    }else{
      OMPL_ERROR("%s: Did not specify optimization function.", getName().c_str());
      exit(0);
    }
  }else{
    setup_ = false;
  }

}

void QuotientChartSubGraph::Init()
{
  //#########################################################################
  //Adding start configuration
  //#########################################################################
  if(const ob::State *state_start = pis_.nextStart()){
    v_start = AddConfiguration(state_start);
    q_start = graph[v_start];
    q_start->isStart = true;
  }else{
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }

  //#########################################################################
  //Adding goal configuration
  //#########################################################################
  if(const ob::State *state_goal = pis_.nextGoal()){
    s_goal = state_goal;
  }else{
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
  OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nearest_configuration->size());
}
void QuotientChartSubGraph::clear()
{
  BaseT::clear();
  if(nearest_configuration) nearest_configuration->clear();
  graph.m_graph.clear();
}
void QuotientChartSubGraph::Rewire()
{
  Vertex v = boost::random_vertex(graph, rng_boost);
  Configuration *q = graph[v];

  std::vector<Configuration*> neighbors;
  //Vertex v = get(normalizedIndexToVertex, q->index);
  uint Nv = boost::degree(v, graph);
  uint K = Nv+2;
  nearest_configuration->nearestK(const_cast<Configuration*>(q), K, neighbors);

  for(uint k = Nv+1; k < neighbors.size(); k++){
    Configuration *qn = neighbors.at(k);
    if(Q1->checkMotion(q->state, qn->state))
    {
      AddEdge(q, qn);
    }
  }
}
void QuotientChartSubGraph::ExtendGraphOneStep()
{
  if(hasSolution){
    //No Goal Biasing if we already found a solution on this quotient space
    Sample(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      Q1->copyState(q_random->state, s_goal);
    }else{
      Sample(q_random->state);
    }
  }

  Configuration *q_nearest = Nearest(q_random);

  double d = Q1->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  if(Q1->checkMotion(q_nearest->state, q_random->state))
  {
    Vertex v = AddConfiguration(q_random->state);
    Configuration *q_next = graph[v];

    AddEdge(q_nearest, q_next);

    if(!hasSolution){
      double dist = 0.0;
      bool satisfied = goal->isSatisfied(q_next->state, &dist);
      if(satisfied)
      {
        v_goal = AddConfiguration(s_goal);
        AddEdge(q_nearest, graph[v_goal]);
        hasSolution = true;
      }
    }
  }
}


QuotientChartSubGraph::Configuration::Configuration(const base::SpaceInformationPtr &si): 
  state(si->allocState())
{}
QuotientChartSubGraph::Configuration::Configuration(const base::SpaceInformationPtr &si, const ob::State *state_): 
  state(si->cloneState(state_))
{}

QuotientChartSubGraph::Edge QuotientChartSubGraph::AddEdge(const Configuration *q_from, const Configuration *q_to)
{
  Vertex v_from = q_from->index;
  Vertex v_to = q_to->index;

  std::pair<Edge, bool> result = boost::add_edge(v_from, v_to, graph);
  double d = Distance(q_from, q_to);
  graph[result.first].setWeight(d);
  return result.first;
}

QuotientChartSubGraph::Vertex QuotientChartSubGraph::AddConfiguration(const ob::State *s)
{
  Vertex m = boost::add_vertex(graph);
  Configuration *q = new Configuration(si_, s);
  graph[m] = q;
  q->index = m;
  nearest_configuration->add(q);
  return m;
}

uint QuotientChartSubGraph::GetNumberOfVertices() const{
  return num_vertices(graph);
}
uint QuotientChartSubGraph::GetNumberOfEdges() const{
  return num_edges(graph);
}

//#############################################################################
// Main Sample Function
//#############################################################################

QuotientChartSubGraph::Configuration* QuotientChartSubGraph::Nearest(Configuration *q) const
{
  return nearest_configuration->nearest(q);
}

bool QuotientChartSubGraph::GetSolution(ob::PathPtr &solution)
{
  //if(!isConnected){
  //  Configuration* qn = Nearest(q_goal);
  //  double d_goal = Distance(qn, q_goal);
  //  if(d_goal < 1e-10){
  //    //v_goal = AddConfiguration(q_goal);
  //    v_goal = AddConfiguration(s_goal);
  //    q_goal = graph[v_goal];
  //    q_goal->isGoal = true;
  //    isConnected = true;
  //  }
  //}
  if(hasSolution){
    auto gpath(std::make_shared<PathGeometric>(Q1));
    shortest_path_start_goal.clear();
    shortest_path_start_goal = GetPathOnGraph(v_start, v_goal);
    gpath->clear();
    for(uint k = 0; k < shortest_path_start_goal.size(); k++){
      Configuration *q = graph[shortest_path_start_goal.at(k)];
      gpath->append(q->state);
    }
    std::cout << "solution has " << shortest_path_start_goal.size() << " states" << std::endl;
    solution = gpath;
    return true;
  }
  return false;
}

const QuotientChartSubGraph::SubGraph& QuotientChartSubGraph::GetGraph() const
{
  return graph;
}

// QuotientChartSubGraph::Configuration* QuotientChartSubGraph::GetStartConfiguration() const
// {
//   return q_start;
// }
// QuotientChartSubGraph::Configuration* QuotientChartSubGraph::GetGoalConfiguration() const
// {
//   return q_goal;
// }

// const QuotientChartSubGraph::PDF& QuotientChartSubGraph::GetPDFAllConfigurations() const
// {
//   return pdf_all_configurations;
// }
// const QuotientChartSubGraph::NearestNeighborsPtr& QuotientChartSubGraph::GetNearestNeighborsVertex() const
// {
//   return nearest_configuration;
// }

// double QuotientChartSubGraph::GetGoalBias() const
// {
//   return goalBias;
// }

struct found_goal {}; // exception for termination

template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};

std::vector<const QuotientChartSubGraph::Configuration*> QuotientChartSubGraph::GetPathOnGraph(const Configuration *q_source, const Configuration *q_sink)
{
  // const Vertex v_source = get(indexToVertex, q_source->index);
  // const Vertex v_sink = get(indexToVertex, q_sink->index);
  const Vertex v_source = q_source->index;
  const Vertex v_sink = q_sink->index;
  std::vector<const Configuration*> q_path;
  if(v_source == v_sink) return q_path;
  std::vector<Vertex> v_path = GetPathOnGraph(v_source, v_sink);
  for(uint k = 0; k < v_path.size(); k++){
    q_path.push_back(graph[v_path.at(k)]);
  }
  return q_path;
}
std::vector<QuotientChartSubGraph::Vertex> QuotientChartSubGraph::GetPathOnGraph(const Vertex& v_source, const Vertex& v_sink)
{
  std::vector<Vertex> path;
  std::vector<Vertex> prev(boost::num_vertices(graph));
  auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost), get(boost::edge_bundle, graph));
  //auto predecessor = boost::make_iterator_property_map(prev.begin(), vertexToIndex);

  //check that vertices exists in graph

  try{
    boost::astar_search(graph, v_source,
                    [this, v_sink](const Vertex &v)
                    {
                        return ob::Cost(Distance(graph[v], graph[v_sink]));
                    },
                    boost::predecessor_map(&prev[0])
                      .weight_map(weight)
                      .visitor(astar_goal_visitor<Vertex>(v_sink))
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
  }catch(found_goal fg){
    for(Vertex v = v_sink;; v = prev[graph[v]->index])
    {
      path.push_back(v);
      if(graph[prev[graph[v]->index]]->index == graph[v]->index)
        break;
    }
    std::reverse(path.begin(), path.end());
  }
  return path;
}


void QuotientChartSubGraph::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << std::endl << " |---- [ChartGraph] has " << boost::num_vertices(graph) << " vertices and " << boost::num_edges(graph) << " edges.";
}

void QuotientChartSubGraph::PrintConfiguration(const Configuration* q) const
{
  Q1->printState(q->state);
}

PlannerDataVertexAnnotated QuotientChartSubGraph::getAnnotatedVertex(const Vertex &vertex) const
{
  ob::State *state = Q1->cloneState(graph[vertex]->state);
  if(!state){
    std::cout << "vertex state does not exists" << std::endl;
    Q1->printState(state);
    exit(0);
  }

  PlannerDataVertexAnnotated pvertex(state);
  pvertex.SetLevel(GetLevel());
  pvertex.SetPath(GetChartPath());

  return pvertex;
}

void QuotientChartSubGraph::getPlannerDataAnnotated(base::PlannerData &data) const
{
  // PlannerDataVertexAnnotated pstart = getAnnotatedVertex(v_start);
  // data.addStartVertex(pstart);

  // if(hasSolution){
  //   PlannerDataVertexAnnotated pgoal = getAnnotatedVertex(v_goal);
  //   data.addGoalVertex(pgoal);
  // }

  // std::cout << std::string(80, '-') << std::endl;
  // std::cout << getName() << std::endl;
  // std::cout << data.numVertices() << "," << data.numEdges() << std::endl;

  std::map<const uint, const ob::State*> indexToStates;

  // {
  //   PlannerDataVertexAnnotated p = getAnnotatedVertex(v);
  //   if(graph[v]->isStart) data.addStartVertex(p);
  //   else if(graph[v]->isGoal) data.addGoalVertex(p);
  //   else data.addVertex(p);
  // }

  // foreach (const Edge e, boost::edges(graph))
  // {
  //   const Vertex v1 = boost::source(e, graph);
  //   const Vertex v2 = boost::target(e, graph);
  //   // const ob::State *s1 = graph[v1]->state;
  //   // const ob::State *s2 = graph[v2]->state;
  //   // PlannerDataVertexAnnotated p1(s1);
  //   // PlannerDataVertexAnnotated p2(s2);
  //   // data.addEdge(p1,p2);
  //   data.addEdge(v1,v2);
  // }
  // std::cout << data.numVertices() << "," << data.numEdges() << std::endl;
  PlannerDataVertexAnnotated pstart = getAnnotatedVertex(v_start);
  indexToStates[graph[v_start]->index] = pstart.getState();
  data.addStartVertex(pstart);

  if(hasSolution){
    PlannerDataVertexAnnotated pgoal = getAnnotatedVertex(v_goal);
    indexToStates[graph[v_goal]->index] = pgoal.getState();
    data.addGoalVertex(pgoal);
  }

  //TODO: goal and start are added two times if chart is local
  foreach( const Vertex v, boost::vertices(graph))
  {
    if(indexToStates.find(graph[v]->index) == indexToStates.end()) {
      PlannerDataVertexAnnotated p = getAnnotatedVertex(v);
      indexToStates[graph[v]->index] = p.getState();
      data.addVertex(p);
    }
    //otherwise vertex is a goal or start vertex and has already been added
  }
  foreach (const Edge e, boost::edges(graph))
  {
    const Vertex v1 = boost::source(e, graph);
    const Vertex v2 = boost::target(e, graph);

    const ob::State *s1 = indexToStates[graph[v1]->index];
    const ob::State *s2 = indexToStates[graph[v2]->index];
    PlannerDataVertexAnnotated p1(s1);
    PlannerDataVertexAnnotated p2(s2);
    data.addEdge(p1,p2);
  }
}

QuotientChartSubGraph::SubGraph& QuotientChartSubGraph::GetSubGraphComponent( uint k_component )
{
  //SubGraph s1 = graph.create_subgraph();
  //boost::add_vertex(v_start, s1);

  SubGraph *subgraph = new SubGraph();
  *subgraph = graph.create_subgraph();

  foreach( const Vertex v, boost::vertices(graph))
  {
    boost::add_vertex(v, *subgraph);
  }
  return *subgraph;
}

void QuotientChartSubGraph::CopyChartFromSibling( QuotientChart *sibling_chart, uint k )
{
  QuotientChartSubGraph *sibling = dynamic_cast<QuotientChartSubGraph*>(sibling_chart);
  //Get k-th subgraph
  graph = sibling->GetSubGraphComponent(k);

  v_start = sibling->v_start;
  v_goal = sibling->v_goal;
  q_start = graph[v_start];
  q_goal = graph[v_goal];
}
