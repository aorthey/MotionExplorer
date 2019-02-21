#include "plannerdata_vertex_annotated.h"

using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;

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
  simplicial_complex_local = rhs.GetComplex();
  infeasible = rhs.IsInfeasible();
  feasibility_t = rhs.GetFeasibility();

  state_quotient_space = rhs.getQuotientState();
}

void PlannerDataVertexAnnotated::SetInfeasible()
{
  infeasible=true;
}
bool PlannerDataVertexAnnotated::IsInfeasible() const
{
  return infeasible;
}
ob::PlannerDataVertex *PlannerDataVertexAnnotated::clone() const 
{
  return new PlannerDataVertexAnnotated(*this);
}

//##############################################################################
void PlannerDataVertexAnnotated::SetFeasibility(FeasibilityType feasibility_t_)
{
  feasibility_t = feasibility_t_;
}
FeasibilityType PlannerDataVertexAnnotated::GetFeasibility() const
{
  return feasibility_t;
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
void PlannerDataVertexAnnotated::AddComplex(std::vector<long unsigned int> simplex_)
{
  simplicial_complex_local.push_back(simplex_);
}
void PlannerDataVertexAnnotated::SetComplex(std::vector<std::vector<long unsigned int>> complex_)
{
  simplicial_complex_local = complex_;
}
std::vector<std::vector<long unsigned int>> PlannerDataVertexAnnotated::GetComplex() const
{
  return simplicial_complex_local;
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
const ob::State *PlannerDataVertexAnnotated::getQuotientState() const
{
  return state_quotient_space;
}
void PlannerDataVertexAnnotated::setQuotientState(const ob::State *s)
{
  state_quotient_space = s;
}
void PlannerDataVertexAnnotated::setState(ob::State *s)
{
  state_ = s;
}
void PlannerDataVertexAnnotated::DrawGL(GUIState& state)
{
  std::cout << "DRAW OPENSET" << std::endl;
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
