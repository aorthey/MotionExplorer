#pragma once
#include "neighborhood.h"
#include <cmath>

struct NeighborhoodSE2: public Neighborhood
{
  NeighborhoodSE2() = default;
  double WorkspaceDistanceToConfigurationSpaceDistance(double d) override
  {
    return 0.63/std::sqrt(2);
  }
};
