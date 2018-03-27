#pragma once
#include "validity_checker_ompl.h"
#include "planner/cover/open_set_convex.h"

class ValidityCheckerSimplicialComplex: public OMPLValidityChecker
{
  public:
    ValidityCheckerSimplicialComplex(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_);
    cover::OpenSetConvex ComputeNeighborhood(const ob::State* state) const;
};
