#include "QuotientGraphSparse.h"
#include "common.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
using namespace og;
#define foreach BOOST_FOREACH

QuotientGraphSparse::QuotientGraphSparse(const ob::SpaceInformationPtr &si, Quotient *parent):
  BaseT(si, parent)
{
  setName("QuotientGraphSparse");
  // specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  // specs_.approximateSolutions = false;
  // specs_.optimizingPaths = false;
  Planner::declareParam<double>("sparse_delta_fraction", this, &QuotientGraphSparse::setSparseDeltaFraction,
                                &QuotientGraphSparse::getSparseDeltaFraction, "0.0:0.01:1.0");

  pathVisibilityChecker_ = new PathVisibilityChecker(si);

  if (!isSetup())
  {
    setup();
  }
}

QuotientGraphSparse::~QuotientGraphSparse()
{
}


void QuotientGraphSparse::DeleteConfiguration(Configuration *q)
{
  BaseT::DeleteConfiguration(q);
}

void QuotientGraphSparse::setup()
{
  BaseT::setup();
  if (!nearestSparse_){
    nearestSparse_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration*>(this));
    nearestSparse_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                               return Distance(a, b);
                             });
  }
  double maxExt = si_->getMaximumExtent();
  sparseDelta_ = sparseDeltaFraction_ * maxExt;
  std::cout << "Sparse delta=" << sparseDelta_ << std::endl;
}

void QuotientGraphSparse::clear()
{
  BaseT::clear();

  graphSparse_.clear();
  if(nearestSparse_) nearestSparse_->clear();

  pathStackHead_.clear();
  pathStack_.clear();
}

void QuotientGraphSparse::Init()
{
  BaseT::Init();
  assert(boost::num_vertices(graphSparse_)==1);
  v_start_sparse = graphSparse_[0]->index;
  graphSparse_[v_start_sparse]->isStart = true;
  v_goal_sparse = AddConfigurationSparse(q_goal);
  graphSparse_[v_goal_sparse]->isGoal = true;
}

QuotientGraphSparse::Vertex QuotientGraphSparse::AddConfiguration(Configuration *q)
{
  Vertex v = BaseT::AddConfiguration(q);

  findGraphNeighbors(q, graphNeighborhood, visibleNeighborhood);

  if(visibleNeighborhood.empty())
  {
    AddConfigurationSparse(q);
  }else{
    if(!checkAddConnectivity(q, visibleNeighborhood)){
      if (!checkAddInterface(q, graphNeighborhood, visibleNeighborhood)){
      }
    }
  }

  return v;
}

QuotientGraphSparse::Vertex QuotientGraphSparse::AddConfigurationSparse(Configuration *q)
{
    Configuration *ql = new Configuration(Q1, q->state);
    const Vertex vl = add_vertex(ql, graphSparse_);
    nearestSparse_->add(ql);
    disjointSets_.make_set(vl);
    graphSparse_[vl]->index = vl;
    return vl;
}

void QuotientGraphSparse::findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                                   std::vector<Configuration*> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    nearestSparse_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration* qn : graphNeighborhood)
        if (Q1->checkMotion(q->state, qn->state))
            visibleNeighborhood.push_back(qn);
}
void QuotientGraphSparse::AddEdgeSparse(const Vertex a, const Vertex b)
{
  ob::Cost weight = opt_->motionCost(graphSparse_[a]->state, graphSparse_[b]->state);
  EdgeInternalState properties(weight);
  boost::add_edge(a, b, properties, graphSparse_);
  uniteComponents(a, b);
}

bool QuotientGraphSparse::checkAddConnectivity(Configuration* q, std::vector<Configuration*> &visibleNeighborhood)
{
    std::vector<Vertex> links;
    if (visibleNeighborhood.size() > 1)
    {
        // For each neighbor
        for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
        {
            // For each other neighbor
            for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
            {
                // If they are in different components
                if (!sameComponent(visibleNeighborhood[i]->index, visibleNeighborhood[j]->index))
                {
                    links.push_back(visibleNeighborhood[i]->index);
                    links.push_back(visibleNeighborhood[j]->index);
                }
            }
        }

        if (!links.empty())
        {
            Vertex v = AddConfigurationSparse(q);

            for (Vertex link : links){
                // If there's no edge
                if (!boost::edge(v, link, graphSparse_).second){
                    // And the components haven't been united by previous links
                    if (!sameComponent(link, v)){
                        // connectGuards(g, link);
                        AddEdgeSparse(v, link);
                    }
                }
            }
            return true;
        }
    }
    return false;
}
bool QuotientGraphSparse::checkAddInterface(Configuration *q,
    std::vector<Configuration*> &graphNeighborhood, 
    std::vector<Configuration*> &visibleNeighborhood)
{
    // If we have more than 1 or 0 neighbors
    if (visibleNeighborhood.size() > 1)
    {
        Configuration *qn0 = graphNeighborhood[0];
        Configuration *qn1 = graphNeighborhood[1];
        Configuration *qv0 = visibleNeighborhood[0];
        Configuration *qv1 = visibleNeighborhood[1];

        if (qn0 == qv0 && qn1 == qv1){
            // If our two closest neighbors don't share an edge
            if (!boost::edge(qv0->index, qv1->index, graphSparse_).second)
            {
                // If they can be directly connected
                if (si_->checkMotion(qv0->state, qv1->state))
                {
                    AddEdgeSparse(qv0->index, qv1->index);
                }else{
                  // Add the new node to the graph, to bridge the interface
                  // Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
                  Vertex v = AddConfigurationSparse(q);
                  AddEdgeSparse(v, qv0->index);
                  AddEdgeSparse(v, qv1->index);
                }
                // Report success
                return true;
            }
        }
    }
    return false;
}



//############################################################################
//############################################################################

void QuotientGraphSparse::Rewire(Vertex &v)
{
  Configuration *q = G[v];
  std::vector<Configuration*> neighbors;
  uint Nv = boost::degree(v, G);
  uint K = Nv+2;
  nearest_datastructure->nearestK(const_cast<Configuration*>(q), K, neighbors);

  for(uint k = Nv+1; k < neighbors.size(); k++){
    Configuration *qn = neighbors.at(k);
    if(Q1->checkMotion(q->state, qn->state))
    {
      AddEdge(q->index, qn->index);
    }
  }
}

void QuotientGraphSparse::Rewire()
{
  Vertex v = boost::random_vertex(G, rng_boost);
  return Rewire(v);
}

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void QuotientGraphSparse::printAllPathsUtil(Vertex u, Vertex d, bool visited[],
                            int path[], int &path_index) 
{
  //terminate if we have enough paths in stack
    if(pathStack_.size() > Nhead) return;

    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    // If current vertex is same as destination, then print
    // current path[]
    if (u == d)
    {
        std::vector<ob::State*> pp;
        for (int i = 0; i<path_index; i++){
            pp.push_back(graphSparse_[path[i]]->state);
        }
        AddPathToStack(pp);
    }
    else // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        OEIterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(u, graphSparse_); ei != ei_end; ++ei) {
          Vertex source = boost::source ( *ei, graphSparse_ );
          Vertex target = boost::target ( *ei, graphSparse_ );
          Vertex vnext = (source==u? target: source);
          if (!visited[vnext]){
              printAllPathsUtil(vnext, d, visited, path, path_index);
          }
        }
        // for (i = adj[u].begin(); i != adj[u].end(); ++i)
        //     if (!visited[*i])
        //         printAllPathsUtil(*i, d, visited, path, path_index);
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}

void QuotientGraphSparse::AddPathToStack(std::vector<ob::State*> &path)
{
  og::PathGeometric gpath(Q1);
  for(uint k = 0; k < path.size(); k++){
    gpath.append(path.at(k));
  }
  og::PathSimplifier shortcutter(Q1);
  shortcutter.simplifyMax(gpath);
  shortcutter.smoothBSpline(gpath);
  // shortcutter.shortcutPath(gpath);

  // PrintPathStack();
  if(pathStack_.empty()){
    pathStack_.push_back(gpath);
  }else{

    for(uint k = 0; k < pathStack_.size(); k++){
      og::PathGeometric& pathk = pathStack_.at(k);
      if(pathVisibilityChecker_->IsPathVisible(gpath.getStates(), pathk.getStates())){
        return;
      }
    }
    pathStack_.push_back(gpath);
  }
  // PrintPathStack();

}
void QuotientGraphSparse::PrintPathStack()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Path Stack" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  for(uint k = 0; k < pathStack_.size(); k++){
    std::vector<ob::State*> pathk = pathStack_.at(k).getStates();
    for(uint j = 0; j < pathk.size(); j++){
      Q1->printState(pathk.at(j));
    }
    std::cout << std::string(80, '-') << std::endl;
  }
}

void QuotientGraphSparse::enumerateAllPaths() 
{
    int V = boost::num_vertices(graphSparse_);
    if(V<=0) return;
    bool *visited = new bool[V];
    std::cout << "Finished planning. Enumerating solution classes" << std::endl;
    std::cout << "Sparse Graph has " << boost::num_vertices(graphSparse_) << " vertices and "
      << boost::num_edges(graphSparse_) << " edges." << std::endl;

    int *path = new int[V];
    int path_index = 0; // Initialize path[] as empty

    for (int i = 0; i < V; i++)
        visited[i] = false;

    printAllPathsUtil(v_start_sparse, v_goal_sparse, visited, path, path_index);

    uint Npathsize = pathStack_.size();
    uint Npaths = std::min(Nhead, Npathsize);
    for(uint k = 0; k < Npaths; k++){
        og::PathGeometric& pathK = (*(pathStack_.rbegin()+k));
        pathStackHead_.push_back(pathK.getStates());
    }
}

void QuotientGraphSparse::getPlannerData(ob::PlannerData &data) const
{

  std::vector<int> NullPath;
  NullPath.push_back(0);
  for(uint k = 0; k < level; k++){
    NullPath.push_back(0);
  }
  // PlannerDataVertexAnnotated pstart(graphSparse_[v_start_sparse]->state);
  // pstart.SetPath(NullPath);
  // data.addStartVertex(pstart);

  // if(hasSolution){
  //   PlannerDataVertexAnnotated pgoal(graphSparse_[v_goal_sparse]->state);
  //   pgoal.SetPath(NullPath);
  //   data.addGoalVertex(pgoal);
  // }

  // std::cout << "Sparse Graph has " << boost::num_vertices(graphSparse_) << " vertices and "
  //   << boost::num_edges(graphSparse_) << " edges." << std::endl;

  foreach (const Vertex v, boost::vertices(graphSparse_))
  {
    PlannerDataVertexAnnotated p(graphSparse_[v]->state);
    p.SetLevel(level);
    p.SetPath(NullPath);

    data.addVertex(p);
  }
  foreach (const Edge e, boost::edges(graphSparse_))
  {
    const Vertex v1 = boost::source(e, graphSparse_);
    const Vertex v2 = boost::target(e, graphSparse_);

    PlannerDataVertexAnnotated p1(graphSparse_[v1]->state);
    PlannerDataVertexAnnotated p2(graphSparse_[v2]->state);

    data.addEdge(p1,p2);
  }

  if(hasSolution){
    //TODO:
    // - display path on different horizontal index
    // - Fix Bug where non-subgraphs are added (after CLEAR command)
    // -- (smooth them somehow simultaneously if a vertex belongs to multiple paths)

    for(uint i = 0; i < pathStackHead_.size(); i++){
        // Vpath.push_back(i);
        NullPath.back() += i;

        // Vertex v1 = pathK.at(0);
        const std::vector<ob::State*> states = pathStackHead_.at(i);
        PlannerDataVertexAnnotated p1(states.at(0));
        p1.SetLevel(level);
        p1.SetPath(NullPath);
        data.addStartVertex(p1);

        for(uint k = 0; k < states.size()-1; k++){
          // std::vec v2 = pathK.at(k+1);

          PlannerDataVertexAnnotated p2(states.at(k+1));//Q1->cloneState(graphSparse_[v2]->state));
          p2.SetLevel(level);
          p2.SetPath(NullPath);

          if(k==states.size()-2){
            data.addGoalVertex(p2);
          }else{
            data.addVertex(p2);
          }
          data.addEdge(p1,p2);

          p1 = p2;
        }
    }
  }
}
