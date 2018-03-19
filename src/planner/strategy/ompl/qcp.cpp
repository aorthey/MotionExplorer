#include "qcp.h"
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
  auto checkerPtr = static_pointer_cast<ValidityCheckerSimplicialComplex>(si_->getStateValidityChecker());
  const ob::State *start = startS_.at(0);
  checkerPtr->ComputeNeighborhood(start);
}

void QCP::CheckForSolution(ob::PathPtr &solution)
{
}
