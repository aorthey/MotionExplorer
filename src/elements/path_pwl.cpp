#include "elements/path_pwl.h"
#include <iostream>
//using namespace std;

PathPiecewiseLinear::PathPiecewiseLinear()
{
}

void PathPiecewiseLinear::Normalize(){
  double newLength =0.0;
  for(uint i = 0; i < interLength.size(); i++){
    interLength.at(i) /= length;
    newLength+=interLength.at(i);

  }
  assert( fabs(newLength-1.0) < 1e-10);
  length = newLength;
}
std::vector<double> PathPiecewiseLinear::GetLengthVector() const{
  return interLength;
}

double PathPiecewiseLinear::GetLength() const{
  return length;
}

Vector3 PathPiecewiseLinear::EvalVec3(const double t) const{
  Config q = Eval(t);
  Vector3 v; v[0] = q(0); v[1] = q(1); v[2] = q(2);
  return v;
}
Vector3 PathPiecewiseLinear::EvalVec3Milestone(const int k) const{
  Config q;
  if(k<0) q= keyframes.front();
  else if(k>Nkeyframes) q= keyframes.back();
  else q = keyframes.at(k);

  Vector3 v; v[0] = q(0); v[1] = q(1); v[2] = q(2);
  return v;
}
Config PathPiecewiseLinear::EvalMilestone(const int k) const{
  if(k<0) return keyframes.front();
  if(k>Nkeyframes) return keyframes.back();
  return keyframes.at(k);
}
Config PathPiecewiseLinear::ConfigOnSegment(int i, int j, double tloc) const{
  Config q1 = keyframes.at(i);
  Config q2 = keyframes.at(j);
  return q1 + tloc*(q2-q1);
}
Config PathPiecewiseLinear::Eval(const double t) const{
  if(t<=0) return keyframes.front();
  if(t>=length) return keyframes.back();
  double Tcum = 0;
  assert(interLength.size()==keyframes.size()-1);
  for(uint i = 0; i < interLength.size(); i++){
    double Tnext = interLength.at(i);
    if((Tcum+Tnext)>=t){
      //t \in [Lcum, Lcum+Lnext]
      double tloc = (t-Tcum)/Tnext; //tloc \in [0,1]
      return ConfigOnSegment(i,i+1,tloc);
    }
    Tcum+=Tnext;
  }
  //rounding errors could lead to the fact that the cumulative length is not
  //exactly 1. If t is sufficiently close, we just return the last keyframe.
  if(length-Tcum > 1e-10){
    std::cout << "length of path is significantly different from 1.0" << std::endl;
    std::cout << "length    : " << Tcum << "/" << 1.0 << std::endl;
    std::cout << "difference: " << 1.0-Tcum << std::endl;
    throw;
  }

  if(t>=Tcum){
    return keyframes.back();
  }

  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;
}
  
void PathPiecewiseLinear::info() const{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Path Length   : " << length << std::endl;
  std::cout << "Path Keyframes: " << Nkeyframes << std::endl;
  std::cout << "Path Dimensionality: " << Ndim << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  double dstep = length/10.0;
  for(double d = 0; d < length; d+=dstep){
    std::cout << Eval(d) << std::endl;  
  }
  std::cout << Eval(length) << std::endl;  
}
//PathPiecewiseLinear* PathPiecewiseLinear::from_keyframes(const std::vector<Config> &keyframes){
//  PathPiecewiseLinear* path = new PathPiecewiseLinear();
//  path->keyframes=keyframes;
//  path->Ndim = keyframes.at(0).size();
//  path->Nkeyframes = keyframes.size();
//  path->interpolate();
//  return path;
//}
//
//PathPiecewiseLinear* PathPiecewiseLinear::from_keyframes(const std::vector<Vector3> &keyframes){
//  PathPiecewiseLinear* path = new PathPiecewiseLinear();
//  for(uint i = 0; i < keyframes.size(); i++){
//    Config q;q.resize(3);q(0)=keyframes.at(i)[0];q(1)=keyframes.at(i)[1];q(2)=keyframes.at(i)[2];
//    path->keyframes.push_back(q);
//  }
//  path->Ndim = 3;
//  path->Nkeyframes = keyframes.size();
//  path->interpolate();
//  return path;
//}
double PathPiecewiseLinear::PosFromConfig(const Config q) const{
  double pos = 0;
  for(uint k = 0; k < keyframes.size()-1; k++){
    Config q1 = keyframes.at(k);
    Config q2 = keyframes.at(k+1);
    double d1 = (q-q1).norm();
    double d2 = (q-q2).norm();
    double d = (q1-q2).norm();
    double epsilon = 1e-10;
    if(d1+d2 <= d+epsilon){
      //q lies on line (q1,q2)
      //
      if(d<epsilon){
        return 0;
      }
      pos += d1/d*interLength.at(k);
      Config qout = Eval(pos);
      assert((qout-q).norm() < epsilon);
      return pos;
    }
    pos += interLength.at(k);

  }

}
