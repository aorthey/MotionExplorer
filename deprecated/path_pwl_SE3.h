#pragma once
#include "elements/path_pwl.h"
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
typedef Math::Vector Config;
using Math3D::Vector3;

// Continuous Piecewise-Linear Path on Euclidean Space without velocity information
//
// [0,L] -> R^n, piecewise linear
// [0,1] -> R^n, piecewise linear (normalized)

// convention
//-3.1415 <= q(3) <= +3.1415
//-1.57   <= q(4) <= +1.57
//-3.1415 <= q(5) <= +3.1415
class PathPiecewiseLinearSE3: public PathPiecewiseLinear
{
  std::vector<bool> boundary_cross_yaw;
  std::vector<bool> boundary_cross_pitch;
  std::vector<bool> boundary_cross_roll;
  public:
    PathPiecewiseLinearSE3();
    virtual void interpolate() override;
    virtual Config ConfigOnSegment(int i, int j, double tloc) const;
    static PathPiecewiseLinearSE3* from_keyframes(const std::vector<Config> &keyframes);
};
