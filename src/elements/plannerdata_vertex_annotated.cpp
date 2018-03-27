#include "plannerdata_vertex_annotated.h"

PlannerDataVertexAnnotated::PlannerDataVertexAnnotated(const ob::State *st, int tag, double d_):
      ob::PlannerDataVertex(st,tag), open_neighborhood_distance(d_)
{
}

PlannerDataVertexAnnotated::PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs): 
  ob::PlannerDataVertex(rhs.state_, rhs.tag_)
{
  open_neighborhood_distance = rhs.GetOpenNeighborhoodDistance();
  level = rhs.GetLevel();
  max_level = rhs.GetMaxLevel();
  component = rhs.GetComponent();
  openset = rhs.GetOpenSet();
}

ob::PlannerDataVertex *PlannerDataVertexAnnotated::clone() const 
{
  return new PlannerDataVertexAnnotated(*this);
}

//##############################################################################
cover::OpenSetConvex PlannerDataVertexAnnotated::GetOpenSet() const
{
  return openset;
}
void PlannerDataVertexAnnotated::SetOpenSet( cover::OpenSetConvex openset_)
{
  openset = openset_;
}

//##############################################################################
double PlannerDataVertexAnnotated::GetOpenNeighborhoodDistance() const
{
  return open_neighborhood_distance;
}

void PlannerDataVertexAnnotated::SetOpenNeighborhoodDistance(double d_)
{
  open_neighborhood_distance = d_;
}

//##############################################################################
void PlannerDataVertexAnnotated::SetComponent(uint component_)
{
  component = component_;
}
uint PlannerDataVertexAnnotated::GetComponent() const
{
  return component;
}

//##############################################################################
void PlannerDataVertexAnnotated::SetLevel(uint level_)
{
  level = level_;
}
uint PlannerDataVertexAnnotated::GetLevel() const
{
  return level;
}

//##############################################################################
void PlannerDataVertexAnnotated::SetMaxLevel(uint level_)
{
  max_level = level_;
}
uint PlannerDataVertexAnnotated::GetMaxLevel() const
{
  return max_level;
}

//##############################################################################
const ob::State *PlannerDataVertexAnnotated::getState() const 
{
  return state_;
}
void PlannerDataVertexAnnotated::setState(ob::State *s)
{
  state_ = s;
}
void PlannerDataVertexAnnotated::DrawGL(GUIState& state)
{
  openset.DrawGL(state);
}

std::ostream& operator<< (std::ostream& out, const PlannerDataVertexAnnotated& v)
{
  out << "AnnotatedVertex";
  out << " ->level " << v.GetLevel() << "/" << v.GetMaxLevel();
  out << " ->component " << v.GetComponent();
  out << std::endl;
  return out;
}
