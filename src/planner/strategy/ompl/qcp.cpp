#include "qcp.h"
#include <ompl/util/Console.h>

using namespace og;
// should include a graph of SC
// can SBL handle N-faces?
QCP::QCP(const ob::SpaceInformationPtr &si, Quotient *previous_):
  Quotient(si, previous_)
{
}

QCP::~QCP()
{
}

void QCP::Init()
{
}

void QCP::Grow(double t)
{
}

void QCP::CheckForSolution(ob::PathPtr &solution)
{
}
