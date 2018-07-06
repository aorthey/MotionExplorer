#include "quotient_chart.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "common.h"
#include <boost/foreach.hpp>

using namespace og;
#define foreach BOOST_FOREACH

QuotientChart::QuotientChart(const ob::SpaceInformationPtr &si, og::QuotientChart *parent_)
  : BaseT(si, parent_)
{
}
void QuotientChart::Grow(double t)
{
  BaseT::Grow(t);
}
double QuotientChart::GetImportance() const
{
  return importance;
}
void QuotientChart::SetImportance(double importance_)
{
  importance = importance_;
}
uint QuotientChart::GetLevel() const
{
  return level;
}
void QuotientChart::SetLevel(uint level_)
{
  level = level_;
}
uint QuotientChart::GetHorizontalIndex() const
{
  return horizontal_index;
}
void QuotientChart::SetHorizontalIndex(uint horizontal_index_)
{
  horizontal_index = horizontal_index_;
}

bool QuotientChart::FoundNewPath()
{
  ob::PathPtr sol;
  if(!hasSolution){
    CheckForSolution(sol);
    if(hasSolution) number_of_paths++;
    return hasSolution;
  }else{
    return false;
  }
}

void QuotientChart::AddSibling(QuotientChart *sibling_)
{
  siblings.push_back(sibling_);
}
void QuotientChart::SetGraph( Graph G_, QuotientChart *sibling)
{
  //G = G_;
  boost::copy_graph( G_, G);
  startM_ = sibling->startM_;
  goalM_ = sibling->goalM_;
  level = sibling->GetLevel();
  number_of_paths = 0;
  local_chart = true;
  disjointSets_ = sibling->disjointSets_;
  nn_ = sibling->nn_;
}

uint QuotientChart::GetNumberOfPaths() const
{
  return number_of_paths;
}

og::QuotientChart::Graph QuotientChart::GetPathSubgraph(uint k)
{
  return G;
}
void QuotientChart::SetChild(QuotientChart *child_)
{
  if(siblings.size()>0) 
  {
    std::cout << "You cannot have a child if you already have siblings." << std::endl;
    exit(0);
  }
  child = child_;
}
uint QuotientChart::GetNumberOfSiblings() const
{
  return siblings.size();
}
void QuotientChart::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();

  //###########################################################################
  //Get Path for this chart
  //###########################################################################
  std::vector<int> path;
  path.push_back(GetHorizontalIndex());
  og::QuotientChart *parent = static_cast<og::QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    path.push_back(parent->GetHorizontalIndex());
    parent = static_cast<og::QuotientChart*>(parent->GetParent());
  }

  std::reverse(std::begin(path), std::end(path));

  //###########################################################################
  //Get Data from this chart
  //###########################################################################
  uint startComponent = 0;//const_cast<QuotientChart *>(this)->disjointSets_.find_set(startM_.at(0));
  uint goalComponent = 1;//const_cast<QuotientChart *>(this)->disjointSets_.find_set(goalM_.at(0));
  for (unsigned long i : startM_)
  {
    startComponent = const_cast<QuotientChart *>(this)->disjointSets_.find_set(i);
    PlannerDataVertexAnnotated pstart(G[i].state, startComponent);
    pstart.SetLevel(level);
    pstart.SetPath(path);
    pstart.SetComponent(0);
    data.addStartVertex(pstart);

  }

  for (unsigned long i : goalM_)
  {
    PlannerDataVertexAnnotated pgoal(G[i].state, const_cast<QuotientChart *>(this)->disjointSets_.find_set(i));
    pgoal.SetLevel(level);
    pgoal.SetPath(path);
    pgoal.SetComponent(1);
    data.addGoalVertex(pgoal);
  }

  std::cout << "vertices " << GetNumberOfVertices() << " edges " << GetNumberOfEdges() << std::endl;
  uint ctr = 0;
  foreach (const Edge e, boost::edges(G))
  {
    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);

    // //dirty hack to make sure that plannerData does not remove some vertices as
    // //duplicated
    //double epsilon = 1e-5;
    //@TODO: put that into Vertexinternalstate

    PlannerDataVertexAnnotated p1(si_->cloneState(G[v1].state));
    PlannerDataVertexAnnotated p2(si_->cloneState(G[v2].state));

    p1.SetLevel(level);
    p1.SetPath(path);
    p2.SetLevel(level);
    p2.SetPath(path);

    uint vi1 = data.addVertex(p1);
    uint vi2 = data.addVertex(p2);

    data.addEdge(p1,p2);
    ctr++;

    uint v1Component = const_cast<QuotientChart *>(this)->disjointSets_.find_set(v1);
    uint v2Component = const_cast<QuotientChart *>(this)->disjointSets_.find_set(v2);
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


  //for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++)
  //{
  //  PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
  //  v.SetLevel(level);
  //  v.SetPath(path);
  //}
  //###########################################################################
  //Get Data From all siblings
  //###########################################################################

  std::cout << "QuotientChart level " << level << " horIndex:" << horizontal_index 
    << " has " << siblings.size() << " siblings| Path: " << path 
    << " vertices: " << data.numVertices() - Nvertices << std::endl;

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}
