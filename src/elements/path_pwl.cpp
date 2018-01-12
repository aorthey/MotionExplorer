#include "elements/path_pwl.h"
#include "planner/cspace/cspace.h"
#include <iostream>
//using namespace std;

PathPiecewiseLinear::PathPiecewiseLinear()
{
}

PathPiecewiseLinear::PathPiecewiseLinear(ob::PathPtr p_, CSpaceOMPL *cspace_):
  path(p_),cspace(cspace_)
{
  isOmpl=true;
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);

  length = gpath.length();
  std::vector<ob::State *> states = gpath.getStates();

  for(uint k = 1; k < states.size(); k++){
    ob::State *s0 = states.at(k-1);
    ob::State *s1 = states.at(k);
    interLength.push_back(gpath.getSpaceInformation()->distance(s0,s1));
  }
}

void PathPiecewiseLinear::Smooth(){
  if(isOmpl){
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);

    og::PathSimplifier shortcutter(gpath.getSpaceInformation());
    shortcutter.simplifyMax(gpath);

    length = gpath.length();
    std::vector<ob::State *> states = gpath.getStates();

    interLength.clear();
    for(uint k = 1; k < states.size(); k++){
      ob::State *s0 = states.at(k-1);
      ob::State *s1 = states.at(k);
      interLength.push_back(gpath.getSpaceInformation()->distance(s0,s1));
    }
  }
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
  if(isOmpl){
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
    ob::SpaceInformationPtr si = gpath.getSpaceInformation();
    std::vector<ob::State *> states = gpath.getStates();


    if(t<=0) return cspace->OMPLStateToConfig(states.front());
    if(t>=length) return cspace->OMPLStateToConfig(states.back());

    double Tcum = 0;
    assert(interLength.size()==keyframes.size()-1);
    for(uint i = 0; i < interLength.size(); i++){
      double Tnext = interLength.at(i);
      if((Tcum+Tnext)>=t){
        //t \in [Lcum, Lcum+Lnext]
        double tloc = (t-Tcum)/Tnext; //tloc \in [0,1]

        ob::State* s1 = states.at(i);
        ob::State* s2 = states.at(i+1);
        ob::State* sm = si->allocState();
        si->getStateSpace()->interpolate(s1,s2,tloc,sm);
        Config qm = cspace->OMPLStateToConfig(sm);
        si->freeState(sm);
        return qm;
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
      return cspace->OMPLStateToConfig(states.back());
    }


  }else{
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

  }
  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;
}

void PathPiecewiseLinear::interpolate(){
  if(isOmpl) exit(0);
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
