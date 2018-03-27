#include "qcp.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/validitychecker/validity_checker_simplicial_complex.h"
#include <ompl/util/Console.h>

using namespace og;
QCP::QCP(const ob::SpaceInformationPtr &si, Quotient *previous_):
  Quotient(si, previous_)
{
}

QCP::~QCP()
{
}

void QCP::Init()
{
  if(const ob::State *st = pis_.nextStart()){
    startS_.push_back(st);
  }
}

void QCP::Grow(double t)
{
}

void QCP::CheckForSolution(ob::PathPtr &solution)
{
}

void QCP::getPlannerData(ob::PlannerData &data) const
{
  auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());
  const ob::State *start = startS_.at(0);
  cover::OpenSetConvex cvxregion = checkerPtr->ComputeNeighborhood(start);

  PlannerDataVertexAnnotated v(start, 0);
  v.SetOpenNeighborhoodDistance(32);
  v.SetOpenSet(cvxregion);

  data.addStartVertex(v);

  //PlannerDataVertexAnnotated *v0 = static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(0));
  //std::cout << v0->GetOpenSet() << std::endl;
}
