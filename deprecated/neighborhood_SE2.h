#pragma once
#include "neighborhood.h"
#include <cmath>
#include <algorithm>

struct NeighborhoodSE2: public Neighborhood
{
  NeighborhoodSE2() = default;
  //should return d*c, whereby c is some constant depending on geometry of
  //robot. In SE(2) case, this depends on the max radius of the smallest
  //circumscribed ball.
  double ComputeNeighborhoodConstant() override
  {
    const double r1 = 1;
    const double r2 = 0.63; //radius from Planar_Lshape_cylinder_outer.urdf
    const double rmin = std::min(r1, r2);
    const double rmax = std::max(r1, r2);
    const double k1 = 1.0/rmin;
    const double k2 = 1.0/rmax;
    return k2*cos(0.5*M_PI - atan(k1/k2));
  }
};
