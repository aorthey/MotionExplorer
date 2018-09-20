#pragma once
#include "neighborhood.h"

struct NeighborhoodRN: public Neighborhood
{
  NeighborhoodRN() = default;
  double WorkspaceDistanceToConfigurationSpaceDistance(double d) override
  {
    return d;
  }
};
