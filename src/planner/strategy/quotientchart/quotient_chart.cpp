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
  uint Nedges = data.numEdges();

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
  std::cout << "vertices " << GetNumberOfVertices() << " edges " << GetNumberOfEdges() << std::endl;

  //if the chart is local, we need to clone new states such that we have
  //duplicate vertices (sometimes charts are overlapping). 
  std::map<const Vertex, ob::State*> vertexToStates;

  uint startComponent = 0;
  uint goalComponent = 1;
  for (Vertex i : startM_)
  {
    startComponent = const_cast<QuotientChart *>(this)->disjointSets_.find_set(i);

    ob::State *s = (local_chart?si_->cloneState(G[i].state):G[i].state);
    vertexToStates[i] = s;

    PlannerDataVertexAnnotated pstart(s, startComponent);
    pstart.SetLevel(level);
    pstart.SetPath(path);
    pstart.SetComponent(startComponent);
    uint ki = data.addStartVertex(pstart);
    std::cout << "ADDED START VERTEX: " << ki << std::endl;
  }

  for (Vertex i : goalM_)
  {
    goalComponent = const_cast<QuotientChart *>(this)->disjointSets_.find_set(i);

    ob::State *s = (local_chart?si_->cloneState(G[i].state):G[i].state);
    vertexToStates[i] = s;

    PlannerDataVertexAnnotated pgoal(s, goalComponent);
    pgoal.SetLevel(level);
    pgoal.SetPath(path);
    pgoal.SetComponent(goalComponent);
    uint kg = data.addGoalVertex(pgoal);
    std::cout << "ADDED GOAL VERTEX: " << kg << std::endl;
  }

  foreach( const Vertex v, boost::vertices(G))
  {
    if(vertexToStates.find(v) == vertexToStates.end()) {
      ob::State *s = (local_chart?si_->cloneState(G[v].state):G[v].state);
      vertexToStates[v] = s;
      PlannerDataVertexAnnotated p(s);
      p.SetLevel(level);
      p.SetPath(path);
      data.addVertex(p);
    }
    //otherwise vertex is a goal or start vertex and has already been added
  }
  foreach (const Edge e, boost::edges(G))
  {
    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);

    ob::State *s1 = vertexToStates[v1];
    ob::State *s2 = vertexToStates[v2];
    PlannerDataVertexAnnotated p1(s1);
    PlannerDataVertexAnnotated p2(s2);

    uint vi1 = data.vertexIndex(p1);
    uint vi2 = data.vertexIndex(p2);
      // ,vi2;
    // if(v1==startM_.at(0)){
      // vi1 = data.addStartVertex(p1);
    // }else{
      // vi1 = data.addVertex(p1);
    // }
    // if(v2==goalM_.at(0)){
      // vi2 = data.addGoalVertex(p2);
    // }else{
      // vi2 = data.addVertex(p2);
    // }

    data.addEdge(p1,p2);

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

  //###########################################################################
  //Get Data From all siblings
  //###########################################################################

  std::cout << "QuotientChart vIdx " << level << " | hIdx " << horizontal_index 
    << " | siblings " << siblings.size() << " | path " << path 
    << " | vertices " << data.numVertices() - Nvertices 
    << " | edges " << data.numEdges() - Nedges
    << std::endl;

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}
