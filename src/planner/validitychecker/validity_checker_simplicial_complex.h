#pragma once
#include "validity_checker_ompl.h"

class ValidityCheckerSimplicialComplex: public OMPLValidityChecker
{
  public:
    ValidityCheckerSimplicialComplex(const ob::SpaceInformationPtr &si, CSpaceOMPL *ompl_space_, CSpace *inner_);
    void ComputeNeighborhood(const ob::State* state) const;
};
