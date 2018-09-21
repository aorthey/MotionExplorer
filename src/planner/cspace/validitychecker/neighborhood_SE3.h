#pragma once
#include "neighborhood.h"
#include <cmath>
#include <algorithm>

struct NeighborhoodSE3: public Neighborhood
{
  NeighborhoodSE3() = default;
  double ComputeNeighborhoodConstant() override
  {
    const double r1 = 1;
    const double r2 = 1.70073513517; //radius from Xshape_sphere_outer.urdf
    const double rmin = std::min(r1, r2);
    const double rmax = std::max(r1, r2);
    const double k1 = 1.0/rmin;
    const double k2 = 1.0/rmax;
    return k2*cos(0.5*M_PI - atan(k1/k2));
  }
};
