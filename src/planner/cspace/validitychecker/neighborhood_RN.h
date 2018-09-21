#pragma once
#include "neighborhood.h"

struct NeighborhoodRN: public Neighborhood
{
  NeighborhoodRN() = default;
  double ComputeNeighborhoodConstant() override
  {
    return 1;
  }
};
