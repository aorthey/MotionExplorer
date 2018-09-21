#pragma once
#include "neighborhood.h"
#include <cmath>
#include <algorithm>

struct NeighborhoodSE2: public Neighborhood
{
  NeighborhoodSE2() = default;
  double WorkspaceDistanceToConfigurationSpaceDistance(double d) override
  {
    const double r1 = 1;
    const double r2 = 1;

    //Depends on robot geometry => this value needs to be obtained from the
    //robot model
    const double r3 = 0.63;

    const double rmin = std::min(std::min(r1,r2),r3);
    const double rmax = std::max(std::max(r1,r2),r3);

    const double k1 = d/rmin;
    const double k2 = d/rmax;

    const double rq = k2*cos(0.5*M_PI - atan(k1/k2));
    return rq;
  }
};
