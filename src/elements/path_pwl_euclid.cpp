#include "elements/path_pwl_euclid.h"
#include <iostream>
//using namespace std;

PathPiecewiseLinearEuclidean::PathPiecewiseLinearEuclidean()
{
}
void PathPiecewiseLinearEuclidean::interpolate(){
  length = 0;
  interLength.clear();
  for(uint i = 0; i < Nkeyframes-1; i++){
    Config q1 = keyframes.at(i);
    Config q2 = keyframes.at(i+1);
    double d = (q1-q2).norm();
    length += d;
    interLength.push_back(d);
  }
}
  
PathPiecewiseLinearEuclidean* PathPiecewiseLinearEuclidean::from_keyframes(const std::vector<Config> &keyframes){
  PathPiecewiseLinearEuclidean* path = new PathPiecewiseLinearEuclidean();
  path->keyframes=keyframes;
  path->Ndim = keyframes.at(0).size();
  path->Nkeyframes = keyframes.size();
  path->interpolate();
  return path;
}

PathPiecewiseLinearEuclidean* PathPiecewiseLinearEuclidean::from_keyframes(const std::vector<Vector3> &keyframes){
  PathPiecewiseLinearEuclidean* path = new PathPiecewiseLinearEuclidean();
  for(uint i = 0; i < keyframes.size(); i++){
    Config q;q.resize(3);q(0)=keyframes.at(i)[0];q(1)=keyframes.at(i)[1];q(2)=keyframes.at(i)[2];
    path->keyframes.push_back(q);
  }
  path->Ndim = 3;
  path->Nkeyframes = keyframes.size();
  path->interpolate();
  return path;
}
