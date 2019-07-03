#include "QuotientGraphSparse.h"
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

QuotientGraphSparse::QuotientGraphSparse(const ob::SpaceInformationPtr &si, Quotient *parent):
  BaseT(si, parent)
{
  setName("QuotientGraphSparse");
  // specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  // specs_.approximateSolutions = false;
  // specs_.optimizingPaths = false;
  Planner::declareParam<double>("sparse_delta_fraction", this, &QuotientGraphSparse::setSparseDeltaFraction,
                                &QuotientGraphSparse::getSparseDeltaFraction, "0.0:0.01:1.0");

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
}

void QuotientGraphSparse::Init()
{
  BaseT::Init();
}

QuotientGraphSparse::Vertex QuotientGraphSparse::AddConfiguration(Configuration *q)
{
  Vertex v = BaseT::AddConfiguration(q);

  std::vector<Configuration*> graphNeighborhood;
  std::vector<Configuration*> visibleNeighborhood;
  findGraphNeighbors(q, graphNeighborhood, visibleNeighborhood);

  if(visibleNeighborhood.empty())
  {
    AddConfigurationSparse(q);
  }else if(checkAddConnectivity(q, visibleNeighborhood)){
  }else{

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
  Q1->printState(graphSparse_[a]->state);
  Q1->printState(graphSparse_[b]->state);
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
void QuotientGraphSparse::getPlannerData(ob::PlannerData &data) const
{
  // BaseT::getPlannerData(data);

  PlannerDataVertexAnnotated pstart(graphSparse_[v_start]->state);
  data.addStartVertex(pstart);

  // if(hasSolution){
  //   PlannerDataVertexAnnotated pgoal(graph[v_goal]->state);
  //   data.addGoalVertex(pgoal);
  // }

  std::cout << "Sparse Graph has " << boost::num_vertices(graphSparse_) << " vertices and "
    << boost::num_edges(graphSparse_) << " edges." << std::endl;
  foreach (const Vertex v, boost::vertices(graphSparse_))
  {
    PlannerDataVertexAnnotated p(graphSparse_[v]->state);
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
}
