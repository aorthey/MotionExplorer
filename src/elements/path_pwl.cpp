#include "elements/path_pwl.h"
#include "planner/cspace/cspace.h"
#include "gui/drawMotionPlanner.h"
#include <iostream>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateSpace.h>
//using namespace std;

PathPiecewiseLinear::PathPiecewiseLinear()
{
  std::cout << "needs conversion to OMPL" << std::endl;
  exit(0);
}

PathPiecewiseLinear::PathPiecewiseLinear(ob::PathPtr p_, CSpaceOMPL *cspace_):
  path(p_),cspace(cspace_)
{
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);

  length = gpath.length();
  std::vector<ob::State *> states = gpath.getStates();

  for(uint k = 0; k < states.size()-1; k++){
    ob::State *s0 = states.at(k);
    ob::State *s1 = states.at(k+1);
    interLength.push_back(gpath.getSpaceInformation()->distance(s0,s1));
  }

  std::cout << "states : " << states.size() << std::endl;
  std::cout << "interLengths : " << interLength.size() << std::endl;
}

void PathPiecewiseLinear::Smooth(){
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
  std::vector<ob::State *> statesB = gpath.getStates();

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

  path = std::make_shared<og::PathGeometric>(gpath);
  std::cout << "Path smoothed (states: " << statesB.size() << " -> " << states.size() << ")" << std::endl;

  //gpath update is not saved in path!
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
  std::cout << "NYI" << std::endl;
  exit(0);
  //if(k<0) return keyframes.front();
  //if(k>Nkeyframes) return keyframes.back();
  //return keyframes.at(k);
}
//Config PathPiecewiseLinear::ConfigOnSegment(int i, int j, double tloc) const{
//  Config q1 = keyframes.at(i);
//  Config q2 = keyframes.at(j);
//  return q1 + tloc*(q2-q1);
//}
Config PathPiecewiseLinear::Eval(const double t) const{
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
  ob::SpaceInformationPtr si = gpath.getSpaceInformation();
  std::vector<ob::State *> states = gpath.getStates();

  if(t<=0) return cspace->OMPLStateToConfig(states.front());
  if(t>=length) return cspace->OMPLStateToConfig(states.back());

  double Tcum = 0;

  assert(interLength.size()==states.size()-1);

  for(uint i = 0; i < interLength.size(); i++){
    // std::cout << std::string(80, '-') << std::endl;
    // std::cout << "i               : " << i << std::endl;
    // std::cout << "interlength size: " << interLength.size() << std::endl;
    // std::cout << "states size     : " << states.size() << std::endl;
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


  //}else{
  //  if(t<=0) return keyframes.front();
  //  if(t>=length) return keyframes.back();
  //  double Tcum = 0;
  //  assert(interLength.size()==keyframes.size()-1);
  //  for(uint i = 0; i < interLength.size(); i++){
  //    double Tnext = interLength.at(i);
  //    if((Tcum+Tnext)>=t){
  //      //t \in [Lcum, Lcum+Lnext]
  //      double tloc = (t-Tcum)/Tnext; //tloc \in [0,1]
  //      return ConfigOnSegment(i,i+1,tloc);
  //    }
  //    Tcum+=Tnext;
  //  }
  //  //rounding errors could lead to the fact that the cumulative length is not
  //  //exactly 1. If t is sufficiently close, we just return the last keyframe.
  //  if(length-Tcum > 1e-10){
  //    std::cout << "length of path is significantly different from 1.0" << std::endl;
  //    std::cout << "length    : " << Tcum << "/" << 1.0 << std::endl;
  //    std::cout << "difference: " << 1.0-Tcum << std::endl;
  //    throw;
  //  }

  //  if(t>=Tcum){
  //    return keyframes.back();
  //  }

  //}
  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;
}

Vector3 getXYZ(ob::State *s, ob::StateSpacePtr space_input){
  double x = 0;
  double y = 0;
  double z = 0;

  ob::StateSpacePtr space;

  //extract first component subspace
  if(!space_input->isCompound()){
    space= space_input;
  }else{
    int subspaces = space_input->as<ob::CompoundStateSpace>()->getSubspaceCount();
    ob::CompoundStateSpace *M1_compound = space_input->as<ob::CompoundStateSpace>();
    const std::vector<ob::StateSpacePtr> decomposed = M1_compound->getSubspaces();
    space = decomposed.front();
  }

  if(space->getType() == ob::STATE_SPACE_SE3){
    const ob::SE3StateSpace::StateType *qomplSE3 = s->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    x = qomplSE3->getX();
    y = qomplSE3->getY();
    z = qomplSE3->getZ();
  }else if(space->getType() == ob::STATE_SPACE_REAL_VECTOR){
    const ob::RealVectorStateSpace::StateType *qomplRn = s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    x = qomplRn->values[0];
    y = qomplRn->values[1];
    z = qomplRn->values[2];
  }else{
    std::cout << "cannot deal with space type" << space->getType() << std::endl;
    std::cout << "please check ompl/base/StateSpaceTypes.h" << std::endl;
    exit(0);
  }
  Vector3 q(x,y,z);
  return q;

}

void PathPiecewiseLinear::DrawGL(GUIState& state)
{
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
  ob::SpaceInformationPtr si = gpath.getSpaceInformation();
  std::vector<ob::State *> states = gpath.getStates();

  ob::StateSpacePtr space = si->getStateSpace();

  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glPushMatrix();

  glPointSize(ptsize);
  glLineWidth(linewidth);
  cLine.setCurrentGL();
  for(uint i = 0; i < states.size()-1; i++){
    ob::State* c1 = states.at(i);
    ob::State* c2 = states.at(i+1);
    Vector3 q1 = getXYZ(c1, space);
    Vector3 q2 = getXYZ(c2, space);
    GLDraw::drawPoint(q1);
    GLDraw::drawLineSegment(q1, q2);
  }
  glPopMatrix();
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
  glLineWidth(1);
}
  
std::ostream& operator<< (std::ostream& out, const PathPiecewiseLinear& pwl) 
{
  out << std::string(80, '-') << std::endl;
  out << "[Path]" << std::endl;
  out << "Path Length   : " << pwl.length << std::endl;
  out << "Path Keyframes: " << pwl.interLength.size()+1 << std::endl;
  double dstep = pwl.length/10.0;
  for(double d = 0; d < pwl.length; d+=dstep){
    out << pwl.Eval(d) << std::endl;  
  }
  out << std::string(80, '-') << std::endl;
  std::cout << "finish" << std::endl;
}
