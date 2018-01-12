#pragma once
#include <ompl/geometric/PathGeometric.h>
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
typedef Math::Vector Config;
using Math3D::Vector3;

// Continuous Piecewise-Linear Path 
//
// [0,L] -> R^n, piecewise linear
// [0,1] -> R^n, piecewise linear (normalized)
namespace ob = ompl::base;
namespace og = ompl::geometric;

class CSpaceOMPL;

class PathPiecewiseLinear
{
  public:
    PathPiecewiseLinear();
    PathPiecewiseLinear(ob::PathPtr p, CSpaceOMPL *cspace);

    virtual void interpolate();
    virtual Config ConfigOnSegment(int i, int j, double tloc) const;
    Config Eval(const double t) const;
    void info() const;

    std::vector<double> GetLengthVector() const;
    double GetLength() const;
    Vector3 EvalVec3(const double t) const;
    Config EvalMilestone(const int k) const;

    // convert path length [0,L] -> [0,1]
    void Normalize();
    void Smooth();

  protected:
    std::vector<Config> keyframes;
    uint Ndim;
    uint Nkeyframes;
    double length;
    std::vector<double> interLength;//interLength(i) length towards next milestone point from q(i)

    CSpaceOMPL *cspace;
    ob::PathPtr path;
    bool isOmpl{false};
};
