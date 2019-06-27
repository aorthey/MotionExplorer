#include "QuotientSubGraph.h"
using namespace og;

QuotientSubGraph::QuotientSubGraph(const ob::SpaceInformationPtr &si, Quotient *parent):
  BaseT(si, parent)
{
  graphSparse_ = graphDense_.create_subgraph();
}

QuotientSubGraph::~QuotientSubGraph()
{
}

void QuotientSubGraph::DeleteConfiguration(Configuration *q)
{
  BaseT::DeleteConfiguration(q);
}
QuotientGraph::Vertex QuotientSubGraph::AddConfiguration(Configuration *q)
{
  Vertex v = BaseT::AddConfiguration(q);
  return v;
}

void QuotientSubGraph::Rewire(Vertex &v)
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

void QuotientSubGraph::Rewire()
{
  Vertex v = boost::random_vertex(G, rng_boost);
  return Rewire(v);
}
