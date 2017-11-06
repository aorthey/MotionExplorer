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
class PathPiecewiseLinearEuclidean: public PathPiecewiseLinear
{
  public:
    PathPiecewiseLinearEuclidean();
    static PathPiecewiseLinearEuclidean* from_keyframes(const std::vector<Config> &keyframes);
    static PathPiecewiseLinearEuclidean* from_keyframes(const std::vector<Vector3> &keyframes);
    virtual void interpolate() override;
};

