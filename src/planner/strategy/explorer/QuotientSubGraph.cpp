#include "QuotientSubGraph.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
using namespace og;
#define foreach BOOST_FOREACH

QuotientSubGraph::QuotientSubGraph(const ob::SpaceInformationPtr &si, Quotient *parent):
  BaseT(si, parent)
{
  graphSparse_ = graphDense_.create_subgraph();
  setName("QuotientSubGraph");
  // specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  // specs_.approximateSolutions = false;
  // specs_.optimizingPaths = false;
  Planner::declareParam<double>("sparse_delta_fraction", this, &QuotientSubGraph::setSparseDeltaFraction,
                                &QuotientSubGraph::getSparseDeltaFraction, "0.0:0.01:1.0");

  if (!isSetup())
  {
    setup();
  }
}

QuotientSubGraph::~QuotientSubGraph()
{
}
QuotientSubGraph::Configuration::Configuration(const base::SpaceInformationPtr &si): 
  state(si->allocState())
{}
QuotientSubGraph::Configuration::Configuration(const base::SpaceInformationPtr &si, const ob::State *state_): 
  state(si->cloneState(state_))
{}

double QuotientSubGraph::Distance(const Configuration* a, const Configuration* b) const
{
  return si_->distance(a->state, b->state);
}

void QuotientSubGraph::DeleteConfiguration(Configuration *q)
{
  if (q != nullptr){
    if (q->state != nullptr){
      Q1->freeState(q->state);
    }
    delete q;
    q = nullptr;
  }
}

void QuotientSubGraph::setup()
{
  if (!nearestDense_){
    nearestDense_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration*>(this));
    nearestDense_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                               return Distance(a, b);
                             });
    nearestSparse_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration*>(this));
    nearestSparse_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                               return Distance(a, b);
                             });
  }
  double maxExt = si_->getMaximumExtent();
  sparseDelta_ = sparseDeltaFraction_ * maxExt;
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

void QuotientSubGraph::clear()
{
  BaseT::clear();

  // foreach (Vertex v, boost::vertices(graphDense_)){
  //   graphDense_[v]->Remove(Q1);
  // }
  for(auto it = graphDense_.m_children.begin(); it != graphDense_.m_children.end(); it++)
  {
    (*it)->m_graph.clear();
  }
  graphDense_.m_graph.clear();
  graphSparse_.m_graph.clear();

  if(nearestDense_) nearestDense_->clear();
  if(nearestSparse_) nearestSparse_->clear();
  vertexToIndexStdMap.clear();
  indexToVertexStdMap.clear();
  index_ctr = 0;
  firstRun = true;
}

const QuotientSubGraph::Configuration* QuotientSubGraph::Nearest(const Configuration* q) const
{
  return nearestDense_->nearest(const_cast<Configuration*>(q));
}

QuotientSubGraph::Vertex QuotientSubGraph::AddConfiguration(Configuration *q, bool force_add_to_sparse)
{
  Vertex v = boost::add_vertex(graphDense_);
  graphDense_[v] = q;
  nearestDense_->add(q);
  q->index = v;
  put(vertexToIndex, v, index_ctr);
  put(indexToVertex, index_ctr, v);
  index_ctr++;

  if(force_add_to_sparse){
    AddConfigurationSparse(v);
  }else{
    AddConfigurationConditionalSparse(v);
  }
  return v;
}

void QuotientSubGraph::AddConfigurationConditionalSparse(const Vertex &v)
{
  std::vector<Configuration*> graphNeighborhood;
  std::vector<Configuration*> visibleNeighborhood;
  Configuration *q = graphDense_[v];
  findGraphNeighbors(q, graphNeighborhood, visibleNeighborhood);

  if(visibleNeighborhood.empty())
  {
    AddConfigurationSparse(v);
  }

}
void QuotientSubGraph::AddConfigurationSparse(const Vertex &v)
{
    const Vertex vl = add_vertex(graphDense_.global_to_local(v), graphSparse_);
    nearestSparse_->add(graphSparse_[vl]);
}

void QuotientSubGraph::findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                                   std::vector<Configuration*> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    nearestSparse_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration* qn : graphNeighborhood)
        if (Q1->checkMotion(q->state, qn->state))
            visibleNeighborhood.push_back(qn);
}


void QuotientSubGraph::AddEdge(const Configuration* q_from, const Configuration* q_to)
{
  // const Vertex a = q1->index;
  // const Vertex b = q2->index;
  // ob::Cost weight = opt_->motionCost(graphDense_[a]->state, graphDense_[b]->state);
  // EdgeInternalState properties(weight);
  // boost::add_edge(a, b, properties, graphDense_);
  Vertex v_from = q_from->index;
  Vertex v_to = q_to->index;

  std::pair<Edge, bool> result = boost::add_edge(v_from, v_to, graphDense_);
  double d = Distance(q_from, q_to);
  graphDense_[result.first].setWeight(d);

  // return result.first;
}

// void QuotientSubGraph::Rewire(Vertex &v)
// {
//   Configuration *q = G[v];
//   std::vector<Configuration*> neighbors;
//   uint Nv = boost::degree(v, G);
//   uint K = Nv+2;
//   nearest_datastructure->nearestK(const_cast<Configuration*>(q), K, neighbors);

//   for(uint k = Nv+1; k < neighbors.size(); k++){
//     Configuration *qn = neighbors.at(k);
//     if(Q1->checkMotion(q->state, qn->state))
//     {
//       AddEdge(q->index, qn->index);
//     }
//   }
// }

// void QuotientSubGraph::Rewire()
// {
//   Vertex v = boost::random_vertex(G, rng_boost);
//   return Rewire(v);
// }
double QuotientSubGraph::GetImportance() const{
  double N = (double)num_vertices(graphDense_);
  return 1.0/(N+1);
}
bool QuotientSubGraph::GetSolution(ob::PathPtr &solution)
{
  if(hasSolution){
    auto gpath(std::make_shared<PathGeometric>(Q1));
    shortest_path_start_goal.clear();
    shortest_path_start_goal = GetPathOnGraph(v_start, v_goal);
    gpath->clear();
    for(uint k = 0; k < shortest_path_start_goal.size(); k++){
      Configuration *q = graphDense_[shortest_path_start_goal.at(k)];
      gpath->append(q->state);
    }
    std::cout << "solution has " << shortest_path_start_goal.size() << " states" << std::endl;
    solution = gpath;
    return true;
  }
  return false;

}

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

std::vector<const QuotientSubGraph::Configuration*> QuotientSubGraph::GetPathOnGraph(const Configuration *q_source, const Configuration *q_sink)
{
  // const Vertex v_source = get(indexToVertex, q_source->index);
  // const Vertex v_sink = get(indexToVertex, q_sink->index);
  const Vertex v_source = q_source->index;
  const Vertex v_sink = q_sink->index;
  std::vector<const Configuration*> q_path;
  if(v_source == v_sink) return q_path;
  std::vector<Vertex> v_path = GetPathOnGraph(v_source, v_sink);
  for(uint k = 0; k < v_path.size(); k++){
    q_path.push_back(graphDense_[v_path.at(k)]);
  }
  return q_path;
}

std::vector<QuotientSubGraph::Vertex> 
  QuotientSubGraph::GetPathOnGraph(const Vertex& v_source, const Vertex& v_intermediate, const Vertex& v_sink)
{
  std::vector<Vertex> p1 = GetPathOnGraph(v_source, v_intermediate);
  std::vector<Vertex> p2 = GetPathOnGraph(v_intermediate, v_sink);
  p1.insert( p1.end(), p2.begin()+1, p2.end() );
  return p1;
}
std::vector<QuotientSubGraph::Vertex> QuotientSubGraph::GetPathOnGraph(const Vertex& v_source, const Vertex& v_sink)
{
  std::vector<Vertex> path;
  auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost), get(boost::edge_bundle, graphDense_));

  std::vector<Vertex> prev(boost::num_vertices(graphDense_));
  auto predecessor = boost::make_iterator_property_map(prev.begin(), vertexToIndex);

  try{
    //boost::astar_search_tree(graph, v_source,
    boost::astar_search(graphDense_, v_source,
                    [this, v_sink](const Vertex &v)
                    {
                        return ob::Cost(Distance(graphDense_[v], graphDense_[v_sink]));
                    },
                    //boost::predecessor_map(&prev[0])
                    boost::predecessor_map(predecessor)
                      .weight_map(weight)
                      .visitor(astar_goal_visitor<Vertex>(v_sink))
                      .vertex_index_map(vertexToIndex)
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
    for(Vertex v = v_sink;; v = prev[get(vertexToIndex, v)])
    {
      path.push_back(v);
      Vertex p = prev[get(vertexToIndex, v)];
      if(get(vertexToIndex, p) == get(vertexToIndex, v)) break;
    }
    std::reverse(path.begin(), path.end());
  }
  if(path.empty()){
    std::cout << "Empty path" << std::endl;
    std::cout << "From" << std::endl;
    Q1->printState(graphDense_[v_source]->state);
    std::cout << "To" << std::endl;
    Q1->printState(graphDense_[v_sink]->state);
    exit(0);
  }
  return path;
}

void QuotientSubGraph::Init()
{
  if(const ob::State *st = pis_.nextStart()){
    if (st != nullptr){
      q_start = new Configuration(Q1, st);
      q_start->isStart = true;
      v_start = AddConfiguration(q_start);
    }
  }
  if (q_start == nullptr){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }

  if(const ob::State *st = pis_.nextGoal()){
    if (st != nullptr){
      q_goal = new Configuration(Q1, st);
      q_goal->isGoal = true;
      v_goal = AddConfiguration(q_goal);
    }
  }
  if (q_goal == nullptr){
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
}

bool QuotientSubGraph::Sample(ob::State *q_random)
{
  if(parent == nullptr){
    Q1_sampler->sampleUniform(q_random);
  }else{
    if(X1_dimension>0)
    {
      X1_sampler->sampleUniform(s_X1_tmp);
      parent->SampleQuotient(s_Q0_tmp);
      MergeStates(s_Q0_tmp, s_X1_tmp, q_random);
    }else{
      parent->SampleQuotient(q_random);
    }
  }
  return true;
}

bool QuotientSubGraph::SampleQuotient(ob::State *q_random_graph)
{
  const Vertex v = boost::random_vertex(graphDense_, rng_boost);
  Q1->getStateSpace()->copyState(q_random_graph, graphDense_[v]->state);
  return true;
}
void QuotientSubGraph::getPlannerData(ob::PlannerData &data) const
{
  BaseT::getPlannerData(data);

  const SubGraph &graph = graphSparse_;

  // Vertex v_start = graphDense_.global_to_local(v_start);
  PlannerDataVertexAnnotated pstart(graph[v_start]->state);
  data.addStartVertex(pstart);

  // PlannerDataVertexAnnotated pgoal(graph[v_goal]->state);
  // data.addGoalVertex(pgoal);

  foreach (const Vertex v, boost::vertices(graph))
  {
    PlannerDataVertexAnnotated p(graph[v]->state);
    data.addVertex(p);
  }
  foreach (const Edge e, boost::edges(graph))
  {
    const Vertex v1 = boost::source(e, graph);
    const Vertex v2 = boost::target(e, graph);

    PlannerDataVertexAnnotated p1(graph[v1]->state);
    PlannerDataVertexAnnotated p2(graph[v2]->state);

    // uint vi1 = data.addVertex(p1);
    // uint vi2 = data.addVertex(p2);

    data.addEdge(p1,p2);
  }
}
