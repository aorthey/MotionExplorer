#pragma once
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
typedef Math::Vector Config;
using Math3D::Vector3;

// Continuous Piecewise-Linear Path 
//
// [0,L] -> R^n, piecewise linear
// [0,1] -> R^n, piecewise linear (normalized)
class PathPiecewiseLinear
{
  protected:
    std::vector<Config> keyframes;
    uint Ndim;
    uint Nkeyframes;
    double length;
    std::vector<double> interLength;//interLength(i) length towards next milestone point from q(i)
    PathPiecewiseLinear();

  public:
    //virtual static PathPiecewiseLinear* from_keyframes(const std::vector<Config> &keyframes) = 0;
    //virtual static PathPiecewiseLinear* from_keyframes(const std::vector<Vector3> &keyframes) = 0;

    virtual void interpolate() = 0;
    virtual Config ConfigOnSegment(int i, int j, double tloc) const;
    Config Eval(const double t) const;
    void info() const;

    std::vector<double> GetLengthVector() const;
    double GetLength() const;
    Vector3 EvalVec3(const double t) const;
    Vector3 EvalVec3Milestone(const int k) const;
    Config EvalMilestone(const int k) const;
    double PosFromConfig(const Config) const;

    // convert path length [0,L] -> [0,1]
    void Normalize();
};
