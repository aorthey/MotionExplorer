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
  path_class = rhs.GetPathClass();
  max_path_class = rhs.GetMaxPathClass();
  path = rhs.GetPath();
}

ob::PlannerDataVertex *PlannerDataVertexAnnotated::clone() const 
{
  return new PlannerDataVertexAnnotated(*this);
}

//##############################################################################
void PlannerDataVertexAnnotated::SetPath(std::vector<int> path_)
{
  path = path_;
}
std::vector<int> PlannerDataVertexAnnotated::GetPath() const
{
  return path;
}
//##############################################################################
cover::OpenSet* PlannerDataVertexAnnotated::GetOpenSet() const
{
  return openset;
}
void PlannerDataVertexAnnotated::SetOpenSet( cover::OpenSet *openset_)
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
void PlannerDataVertexAnnotated::SetPathClass(uint path_class_)
{
  path_class = path_class_;
}
uint PlannerDataVertexAnnotated::GetPathClass() const
{
  return path_class;
}

//##############################################################################
void PlannerDataVertexAnnotated::SetMaxPathClass(uint max_path_class_)
{
  max_path_class = max_path_class_;
}
uint PlannerDataVertexAnnotated::GetMaxPathClass() const
{
  return max_path_class;
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
  if(openset != nullptr) openset->DrawGL(state);
}

std::ostream& operator<< (std::ostream& out, const PlannerDataVertexAnnotated& v)
{
  out << "AnnotatedVertex";
  out << " ->level " << v.GetLevel() << "/" << v.GetMaxLevel();
  out << " ->component " << v.GetComponent();
  out << std::endl;
  return out;
}
