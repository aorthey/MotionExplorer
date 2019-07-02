#include "QuotientSubGraph.h"
#include <ompl/tools/config/SelfConfig.h>
using namespace og;

QuotientSubGraph::QuotientSubGraph(const ob::SpaceInformationPtr &si, Quotient *parent):
  BaseT(si, parent)
{
  graphSparse_ = graphDense_.create_subgraph();
  setName("QuotientSubGraph");
  specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = false;

  if (!isSetup())
  {
    setup();
  }
}

QuotientSubGraph::~QuotientSubGraph()
{
}

void QuotientSubGraph::DeleteConfiguration(Configuration *q)
{
  BaseT::DeleteConfiguration(q);
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
}

QuotientGraph::Vertex QuotientSubGraph::AddConfiguration(Configuration *q)
{
  Vertex m = boost::add_vertex(q, graphDense_);
  G[m]->total_connection_attempts = 1;
  G[m]->successful_connection_attempts = 0;
  nearestDense_->add(q);
  q->index = m;
  AddConfigurationConditionalSparse(m);
  return m;
}

void QuotientSubGraph::AddConfigurationConditionalSparse(const Vertex &v)
{
  add_vertex(graphDense_.global_to_local(v), graphSparse_);
  nearestSparse_->add(graphDense_[v]);
}


void QuotientSubGraph::AddEdge(const Configuration* q1, const Configuration* q2)
{
  const Vertex a = q1->index;
  const Vertex b = q2->index;
  ob::Cost weight = opt_->motionCost(graphDense_[a]->state, graphDense_[b]->state);
  EdgeInternalState properties(weight);
  boost::add_edge(a, b, properties, graphDense_);
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
