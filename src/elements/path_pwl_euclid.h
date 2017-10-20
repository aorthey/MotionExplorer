#pragma once
#include <Library/KrisLibrary/math/vector.h>
#include <Library/KrisLibrary/math3d/primitives.h>
typedef Math::Vector Config;
using Math3D::Vector3;

// Continuous Piecewise-Linear Path on Euclidean Space without velocity information
//
// [0,L] -> R^n, piecewise linear
// [0,1] -> R^n, piecewise linear (normalized)
class PathPiecewiseLinearEuclidean
{

  protected:
    std::vector<Config> keyframes;
    uint Ndim;
    uint Nkeyframes;
    double length;
    std::vector<double> interLength;//interLength(i) length towards next milestone point from q(i)
    PathPiecewiseLinearEuclidean();

  public:
    //static PathPiecewiseLinearEuclidean from_keyframes(const std::vector<Config> &keyframes);
    //static PathPiecewiseLinearEuclidean from_keyframes(const std::vector<Vector3> &keyframes);
    static PathPiecewiseLinearEuclidean* from_keyframes(const std::vector<Config> &keyframes);
    static PathPiecewiseLinearEuclidean* from_keyframes(const std::vector<Vector3> &keyframes);

    std::vector<double> GetLengthVector() const;
    double GetLength() const;
    Vector3 EvalVec3(const double t) const;
    Vector3 EvalVec3Milestone(const int k) const;
    Config EvalMilestone(const int k) const;
    Config Eval(const double t) const;
    double PosFromConfig(const Config) const;
    virtual void info() const;

    // convert path length [0,L] -> [0,1]
    void Normalize();
    virtual void interpolate();
};

//inline PathPiecewiseLinearEuclidean PathPiecewiseLinearEuclidean::from_keyframes(const std::vector<Config> &keyframes){
//  PathPiecewiseLinearEuclidean path = PathPiecewiseLinearEuclidean();
//  path.keyframes=keyframes;
//  path.Ndim = keyframes.at(0).size();
//  path.Nkeyframes = keyframes.size();
//  path.interpolate();
//  return path;
//}
//inline PathPiecewiseLinearEuclidean PathPiecewiseLinearEuclidean::from_keyframes(const std::vector<Vector3> &keyframes){
//  PathPiecewiseLinearEuclidean path = PathPiecewiseLinearEuclidean();
//  for(int i = 0; i < keyframes.size(); i++){
//    //Vector3 x = keyframes.at(i);
//    Config q;q.resize(3);q(0)=keyframes.at(i)[0];q(1)=keyframes.at(i)[1];q(2)=keyframes.at(i)[2];
//    path.keyframes.push_back(q);
//  }
//  //path.keyframes=keyframes;
//  path.Ndim = 3;
//  path.Nkeyframes = keyframes.size();
//  path.interpolate();
//  return path;
//}
