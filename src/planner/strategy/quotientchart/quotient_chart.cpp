#include "quotient_chart.h"
#include "elements/plannerdata_vertex_annotated.h"

using namespace og;

QuotientChart::QuotientChart(const ob::SpaceInformationPtr &si, og::QuotientChart *parent_)
  : BaseT(si, parent_)
{
}
void QuotientChart::Grow(double t)
{
  BaseT::Grow(t);
}
double QuotientChart::GetImportance()
{
  return importance;
}
void QuotientChart::SetImportance(double importance_)
{
  importance = importance_;
}
uint QuotientChart::GetLevel()
{
  return level;
}
void QuotientChart::SetLevel(uint level_)
{
  level = level_;
}
uint QuotientChart::GetHorizontalIndex()
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
  CheckForSolution(sol);
  return hasSolution;
}
void QuotientChart::AddSibling(QuotientChart *sibling_)
{
  siblings.push_back(sibling_);
}
//QuotientChart::Graph QuotientChart::GetPathSubgraph(uint k)
//{
//  std::cout << "NYI" << std::endl;
//  exit(0);
//  return G;
//}
void QuotientChart::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  BaseT::getPlannerData(data);

  std::vector<int> path;
  og::QuotientChart *parent = static_cast<og::QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    path.push_back(parent->GetHorizontalIndex());
    parent = static_cast<og::QuotientChart*>(parent->GetParent());
  }

  for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++)
  {
    PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
    v.SetLevel(level);
    v.SetPath(path);
  }

  for(uint i = 0; i < siblings.size(); i++){
    siblings.at(i)->getPlannerData(data);
  }
}
