#include "elements/path_pwl.h"
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/cspace/cspace_multiagent.h"
#include "gui/drawMotionPlanner.h"
#include <iostream>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/math/constants/constants.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace oc = ompl::control;

PathPiecewiseLinear::PathPiecewiseLinear(CSpaceOMPL *cspace_):
  cspace(cspace_), quotient_space(cspace_)
{
  SetDefaultPath();
}
void PathPiecewiseLinear::SetDefaultPath()
{
  ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();
  if(quotient_space->isDynamic()){
    path = std::make_shared<oc::PathControl>(si);
  }else{
    path = std::make_shared<og::PathGeometric>(si);
  }
}

PathPiecewiseLinear::PathPiecewiseLinear(ob::PathPtr p_, CSpaceOMPL *cspace_, CSpaceOMPL *quotient_space_):
  cspace(cspace_), quotient_space(quotient_space_), path(p_), path_raw(p_)
{
  if(p_ == nullptr)
  {
    SetDefaultPath();
  }else{

    if(!quotient_space->isDynamic()){

      og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
      length = gpath.length();
      std::vector<ob::State *> states = gpath.getStates();

      uint Nstates = std::max(0,(int)states.size()-1);
      for(uint k = 0; k < Nstates; k++){
        ob::State *s0 = states.at(k);
        ob::State *s1 = states.at(k+1);
        interLength.push_back(gpath.getSpaceInformation()->distance(s0,s1));
      }

    }else{

      oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
      std::vector<ob::State *> states = cpath.getStates();

      uint Nstates = std::max(0,(int)states.size()-1);
      ob::State *x0prime = quotient_space->SpaceInformationPtr()->allocState();
      ob::State *x1prime = quotient_space->SpaceInformationPtr()->allocState();

      for(uint k = 0; k < Nstates; k++){
        ob::State *s0 = states.at(k);
        ob::State *s1 = states.at(k+1);

        Config v0 = quotient_space->OMPLStateToVelocity(s0);
        Config v1 = quotient_space->OMPLStateToVelocity(s1);

        Config x0 = quotient_space->OMPLStateToConfig(s0);
        Config x1 = quotient_space->OMPLStateToConfig(s1);

        Config zeroVel(v0); zeroVel.setZero();
        quotient_space->ConfigVelocityToOMPLState(x0, zeroVel, x0prime);
        quotient_space->ConfigVelocityToOMPLState(x1, zeroVel, x1prime);

        double d = cpath.getSpaceInformation()->distance(x0prime, x1prime);
        interLength.push_back(d);
        length+=d;
      }
    }
  }
}

ob::PathPtr PathPiecewiseLinear::GetOMPLPath() const
{
  return path;
}

void PathPiecewiseLinear::SendToController(SmartPointer<RobotController> controller)
{
  if(!quotient_space->isDynamic()){
    std::cout << "Path is not dynamic, cannot access torques" << std::endl;
    return;
  }

  std::cout << "SENDING CONTROLS" << std::endl;
  std::vector<string> cmds = controller->Commands();
  for(uint k = 0; k < cmds.size(); k++){
    std::cout << cmds.at(k) << std::endl;
  }

  oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
  std::vector<ob::State*> states = cpath.getStates();

  
  // std::vector<oc::Control*> controls = cpath.getControls();
  uint N = quotient_space->GetKlamptDimensionality();
  for(uint k = 0; k < states.size(); k++){
    ob::State *sk = states.at(k);
    Config qk = quotient_space->OMPLStateToConfig(sk);
    Config dqk = quotient_space->OMPLStateToVelocity(sk);

    // std::string sq;
    stringstream qstr, dqstr;
    qstr << N << "  ";
    dqstr << N << "  ";
    for(uint i = 0; i < N; i++){
      qstr<< qk[i] << " ";
      dqstr<< dqk[i] << " ";
    }

    std::cout << qstr.str() << ":::" << dqstr.str() << std::endl;
    string cmd( (k<=0)?("set_q"):("append_q") );
    // string dcmd( (k<=0)?("set_qv"):("append_qv") );

    if(!controller->SendCommand(cmd, qstr.str()))
    {
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "ERROR in controller commander" << std::endl;
      std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      throw "Controller command not supported!";
    }
  }



  // oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
  
  // std::vector<oc::Control*> controls = cpath.getControls();

  // uint N = quotient_space->GetControlDimensionality();
  // for(uint k = 0; k < controls.size(); k++){
  //   const oc::RealVectorControlSpace::ControlType *Rctrl = controls.at(k)->as<oc::RealVectorControlSpace::ControlType>();
  //   stringstream qstr;
  //   qstr << N << "  ";
  //   for(uint i = 0; i < N; i++){
  //     qstr<< Rctrl->values[i] << " ";
  //   }
  //   string cmd( (k<=0)?("set_torque_control"):("append_torque_control") );
  //   if(!controller->SendCommand(cmd,qstr.str())) {
  //     std::cout << std::string(80, '-') << std::endl;
  //     std::cout << "ERROR in controller commander" << std::endl;
  //     std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
  //     std::cout << std::string(80, '-') << std::endl;
  //     throw "Controller command not supported!";
  //   }
  // }

}

void PathPiecewiseLinear::setColor(const GLColor &color)
{
    this->cSmoothed = color;
    this->cUnsmoothed = color;
    this->cVertex = color;
    this->cLine = color;
}

void PathPiecewiseLinear::Smooth(bool forceSmoothing){
  if(path == nullptr) return;
  if(quotient_space->isDynamic()) return;

  if(!isSmooth || forceSmoothing){

    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
    std::vector<ob::State *> statesB = gpath.getStates();

    og::PathSimplifier shortcutter(gpath.getSpaceInformation());
    shortcutter.simplifyMax(gpath);
    shortcutter.smoothBSpline(gpath);

    gpath.interpolate();

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
    isSmooth = true;
  }

  //gpath update is not saved in path!
}
void PathPiecewiseLinear::Normalize(){
  if(!path) return;

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

Vector3 PathPiecewiseLinear::EvalVec3(const double t, int ridx) const{
  Config q = Eval(t);
  ob::ScopedState<> s = quotient_space->ConfigToOMPLState(q);
  Vector3 v = quotient_space->getXYZ(s.get(), ridx);
  if(draw_planar) v[2] = zOffset;
  return v;
}

Vector3 PathPiecewiseLinear::EvalVec3(const double t) const{
  Config q = Eval(t);
  ob::ScopedState<> s = quotient_space->ConfigToOMPLState(q);
  Vector3 v = quotient_space->getXYZ(s.get());
  if(draw_planar) v[2] = zOffset;
  return v;
}

Config PathPiecewiseLinear::EvalStates(std::vector<ob::State*> states, const double t) const{
  ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();

  if(t<=0){
    return quotient_space->OMPLStateToConfig(states.front());
  }
  if(t>=length){
    return quotient_space->OMPLStateToConfig(states.back());
  }

  double Tcum = 0;

  assert(interLength.size()==states.size()-1);

  for(uint i = 0; i < interLength.size(); i++){
    double Tnext = interLength.at(i);
    if((Tcum+Tnext)>=t){
      //t \in [Lcum, Lcum+Lnext]
      double tloc = (t-Tcum)/Tnext; //tloc \in [0,1]
      ob::State* s1 = states.at(i);
      ob::State* s2 = states.at(i+1);
      ob::State* sm = si->allocState();
      si->getStateSpace()->interpolate(s1,s2,tloc,sm);
      Config q = quotient_space->OMPLStateToConfig(sm);
      si->freeState(sm);
      return q;
    }
    Tcum+=Tnext;
  }
  //rounding errors could lead to the fact that the cumulative length is not
  //exactly 1. If t is sufficiently close, we just return the last keyframe.
  double epsilon = 1e-10;
  if(length-Tcum > epsilon){
    std::cout << "length of path is significantly different from " << length << std::endl;
    std::cout << "length    : " << Tcum << "/" << length << std::endl;
    std::cout << "difference: " << length-Tcum << " > " << epsilon << std::endl;
    std::cout << "choosen t : " << t << std::endl;
    OMPL_ERROR("Length of path different from computed length.");
    exit(0);
  }

  if(t>=Tcum){
    return quotient_space->OMPLStateToConfig(states.back());
  }
  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;
}

Config PathPiecewiseLinear::Eval(const double t) const{
  if(!path){
    std::cout << "Cannot Eval empty path" << std::endl;
    throw "Empty path";
  }

  std::vector<ob::State *> states;
  if(quotient_space->isDynamic()){
    oc::PathControl *cpath = static_cast<oc::PathControl*>(path.get());
    states = cpath->getStates();
  }else{
    og::PathGeometric *gpath = static_cast<og::PathGeometric*>(path.get());
    states = gpath->getStates();
  }
  return EvalStates(states, t);
}

Config PathPiecewiseLinear::EvalVelocity(const double t) const{
  if(!quotient_space->isDynamic()) 
  {
    OMPL_ERROR("Cannot eval velocity.");
    throw "EvalVelocityError";
    // Config dq = quotient_space->RobotPtr()->dq;
    // dq.setZero();
    // return dq;
  }

  oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
  std::vector<ob::State *> states = cpath.getStates();

  ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();

  if(t<=0){
    return quotient_space->OMPLStateToVelocity(states.front());
  }
  if(t>=length){
    return quotient_space->OMPLStateToVelocity(states.back());
  }

  double Tcum = 0;

  assert(interLength.size()==states.size()-1);

  for(uint i = 0; i < interLength.size(); i++){
    double Tnext = interLength.at(i);
    if((Tcum+Tnext)>=t){
      //t \in [Lcum, Lcum+Lnext]
      double tloc = (t-Tcum)/Tnext; //tloc \in [0,1]
      ob::State* s1 = states.at(i);
      ob::State* s2 = states.at(i+1);
      ob::State* sm = si->allocState();
      si->getStateSpace()->interpolate(s1,s2,tloc,sm);
      Config q = quotient_space->OMPLStateToVelocity(sm);
      si->freeState(sm);
      return q;
    }
    Tcum+=Tnext;
  }
  //rounding errors could lead to the fact that the cumulative length is not
  //exactly 1. If t is sufficiently close, we just return the last keyframe.
  double epsilon = 1e-10;
  if(length-Tcum > epsilon){
    std::cout << "length of path is significantly different from " << length << std::endl;
    std::cout << "length    : " << Tcum << "/" << length << std::endl;
    std::cout << "difference: " << length-Tcum << " > " << epsilon << std::endl;
    std::cout << "choosen t : " << t << std::endl;
    throw;
  }

  if(t>=Tcum){
    return quotient_space->OMPLStateToVelocity(states.back());
  }

  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;
}

Vector3 PathPiecewiseLinear::Vector3FromState(ob::State *s){
  Vector3 v = quotient_space->getXYZ(s);
  if(draw_planar){
    v[2] = 0.0;
  }
  v[2] += zOffset;
  return v;
}

Vector3 PathPiecewiseLinear::Vector3FromState(ob::State *s, int ridx){
  if(!quotient_space->isMultiAgent()) return Vector3FromState(s);

  Vector3 v = quotient_space->getXYZ(s, ridx);
  if(draw_planar){
    v[2] = 0.0;
  }
  v[2] += zOffset;
  return v;
}


void PathPiecewiseLinear::Draw2DArrow(Vector3 arrow_pos, Vector3 arrow_dir, double arrow_size_head, double arrow_size_length)
{
  Vector3 ez(0,0,1);
  Vector3 onormal = cross(ez, arrow_dir);
  onormal.inplaceNormalize();

  Vector3 o = 0.5*arrow_size_head*onormal;
  //############################################################################
  //Arrow border
  //############################################################################
  black.setCurrentGL();
  glBegin(GL_LINE_STRIP);
  Vector3 intersector = 0.5*linewidth*onormal;
  glVertex3f(intersector[0],intersector[1],intersector[2]);
  glVertex3f(o[0],o[1],o[2]);
  glVertex3f(arrow_dir[0], arrow_dir[1], arrow_dir[2]);
  glVertex3f(-o[0],-o[1],-o[2]);
  glVertex3f(-intersector[0],-intersector[1],-intersector[2]);
  glEnd();

  //############################################################################
  //Fill arrow
  //############################################################################
  cLine.setCurrentGL();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_TRIANGLES);

  glVertex3f(0,0,0);
  glVertex3f(o[0],o[1],o[2]);
  glVertex3f(arrow_dir[0], arrow_dir[1], arrow_dir[2]);

  glVertex3f(0,0,0);
  glVertex3f(-o[0],-o[1],-o[2]);
  glVertex3f(arrow_dir[0], arrow_dir[1], arrow_dir[2]);

  glEnd();
}

Vector3 PathPiecewiseLinear::GetNearestStateToTipOfArrow(Vector3 arrow_pos, 
    const std::vector<ob::State*> states, uint k_start_state, double arrow_size_length, int ridx)
{
    unsigned Mmax = states.size() - 1;
    uint m = k_start_state;

    assert(m < Mmax);

    Vector3 qnext = Vector3FromState(states.at(m), ridx);
    double zmax = qnext[2];
    double d_tip_to_state_best = fabs( arrow_pos.distanceSquared(qnext) - arrow_size_length);

    m = m+1;
    qnext = Vector3FromState(states.at(m), ridx);
    if(qnext[2] > zmax) zmax = qnext[2];
    double d_tip_to_state_next = fabs( arrow_pos.distanceSquared(qnext) - arrow_size_length);

    while(d_tip_to_state_next < d_tip_to_state_best){
      d_tip_to_state_best = d_tip_to_state_next;

      m = m+1;
      if(m>=Mmax){
        m = Mmax;
        break;
      }
      qnext = Vector3FromState(states.at(m), ridx);
      if(qnext[2] > zmax) zmax = qnext[2];
      d_tip_to_state_next = fabs( arrow_pos.distanceSquared(qnext) - arrow_size_length);
    }
    m = m-1;
    
    if(m >= Mmax){
        m = Mmax;
    }
    qnext = Vector3FromState(states.at(m), ridx);
    qnext[2] = zmax;
    return qnext;
}

void PathPiecewiseLinear::DrawGLRibbonRobotIndex(const std::vector<ob::State*> &states, int ridx)
{
  glBegin(GL_QUAD_STRIP);
  std::vector<Vector3> path_left;
  std::vector<Vector3> path_right;
  for(uint i = 0; i < states.size(); i++){
    Vector3 q1 = Vector3FromState(states.at(i), ridx);
    Vector3 dq;

    if(i<states.size()-1){
      Vector3 q2 = Vector3FromState(states.at(i+1), ridx);
      dq = q2 - q1;
    }else{
      Vector3 q2 = Vector3FromState(states.at(i-1), ridx);
      dq = q1 - q2;
    }

    Vector3 dqn;
    dqn[0] = dq[1];
    dqn[1] = -dq[0];
    dqn[2] = 0;
    dqn.inplaceNormalize();
    dqn.inplaceMul(0.5*linewidth);

    Vector3 n = -cross(dq, dqn);
    Matrix3 R0, R1;
    AngleAxisRotation Raa(0, dq);
    Raa.getMatrix(R0);
    AngleAxisRotation Rab(M_PI, dq);
    Rab.getMatrix(R1);

    Vector3 qq1;
    R0.mulTranspose(dqn, qq1);
    glNormal3f(n[0], n[1], n[2]);
    qq1 += q1;
    glVertex3f(qq1[0], qq1[1], qq1[2]);
    path_left.push_back(qq1);

    R1.mulTranspose(dqn, qq1);
    glNormal3f(n[0], n[1], n[2]);
    qq1 += q1;
    glVertex3f(qq1[0], qq1[1], qq1[2]);
    path_right.push_back(qq1);
  }
  glEnd();

  //############################################################################
  //Draws a border around quad strip
  //############################################################################
  // GLfloat sizes[2];  // Store supported line width range
  // GLfloat step;     // Store supported line width increments

  // glGetFloatv(GL_LINE_WIDTH_RANGE,sizes);
  // glGetFloatv(GL_LINE_WIDTH_GRANULARITY,&step);
  // std::cout << step << std::endl;
  // std::cout << sizes[0] << std::endl;
  // std::cout << sizes[1] << std::endl;

  black.setCurrentGL();
  glLineWidth(widthBorder);
  glBegin(GL_LINE_STRIP);
  for(uint k = 0; k < path_left.size(); k++){
    Vector3 v = path_left.at(k);
    glVertex3f(v[0], v[1], v[2]);
  }
  glEnd();
  glBegin(GL_LINE_STRIP);
  for(uint k = 0; k < path_right.size(); k++){
    Vector3 v = path_right.at(k);
    glVertex3f(v[0], v[1], v[2]);
  }
  glEnd();
  cLine.setCurrentGL();
}

void PathPiecewiseLinear::DrawGLRibbon(const std::vector<ob::State*> &states)
{
  //############################################################################
  //Draws a tron-like line strip
  //############################################################################

  if(quotient_space->isMultiAgent()){
    CSpaceOMPLMultiAgent *cma = static_cast<CSpaceOMPLMultiAgent*>(quotient_space);
    std::vector<int> idxs = cma->GetRobotIdxs();
    bool drawMACross = drawCross;
    foreach(int i, idxs)
    {
        DrawGLRibbonRobotIndex(states, i);
        DrawGLArrowMiddleOfPath(states, i);
        if(drawMACross){
          DrawGLCross(states, i);
          drawMACross = false;
        }
    }
    if(!quotient_space->IsPlanar()){
    }else{
    }
  }else{
    DrawGLRibbonRobotIndex(states, quotient_space->GetRobotIndex());
    DrawGLArrowMiddleOfPath(states, quotient_space->GetRobotIndex());
    if(drawCross) DrawGLCross(states, quotient_space->GetRobotIndex());
  }
}


void PathPiecewiseLinear::DrawGLArrowMiddleOfPath( const std::vector<ob::State*> &states, int ridx)
{
  if(states.size() < 2) return;

  Vector3 arrow_pos, arrow_dir;
  double arrow_size_head = 2*linewidth;
  double arrow_size_length = 1.5*arrow_size_head;

  glLineWidth(arrow_size_head*10);
  Vector3 qnext;
  if(states.size() == 2){
    Vector3 q1 = Vector3FromState(states.at(0), ridx);
    qnext = Vector3FromState(states.at(1), ridx);
    arrow_pos = 0.5*(qnext - q1);
  }else if(states.size()%2 == 0){
    unsigned m = 0.5*states.size();
    Vector3 q1 = Vector3FromState(states.at(m-1), ridx);
    Vector3 q2 = Vector3FromState(states.at(m), ridx);
    arrow_pos = q1 + 0.5*(q2 - q1);
    qnext = GetNearestStateToTipOfArrow(arrow_pos, states, m, arrow_size_length, ridx);
  }else{
    unsigned m = floor(0.5*states.size());
    arrow_pos = Vector3FromState(states.at(m), ridx);
    qnext = GetNearestStateToTipOfArrow(arrow_pos, states, m, arrow_size_length, ridx);
  }
  arrow_dir = qnext - arrow_pos;
  arrow_dir.inplaceNormalize();
  arrow_dir.inplaceMul(arrow_size_length);

  //############################################################################
  //Draw Arrow at middle of path
  //############################################################################
  glPushMatrix();
  glTranslatef(arrow_pos[0], arrow_pos[1], arrow_pos[2]+std::max(0.5*zOffset, 1e-3));
  Draw2DArrow(arrow_pos, arrow_dir, arrow_size_head, arrow_size_length);
  glPopMatrix();
}

void PathPiecewiseLinear::DrawGLCross( const std::vector<ob::State*> &states, int ridx)
{
  if(states.size() < 2) return;

  Vector3 ez(0,0,1);

  Vector3 pos = EvalVec3(0.3*GetLength(), ridx);

  //############################################################################
  //Draw Cross at pos
  //############################################################################
  double radius = 1.1*linewidth;

  double barWidth = 0.2*radius;
  double barLength = 0.8*radius;
  double cylinderHeight = 0.05;

  double barOffset = cylinderHeight + std::max(0.5*zOffset, 1e-3);
  Vector3 b1(pos[0]+barLength,pos[1]-barWidth,pos[2]+barOffset);
  Vector3 b2(pos[0]+barLength,pos[1]+barWidth,pos[2]+barOffset);
  Vector3 b3(pos[0]-barLength,pos[1]-barWidth,pos[2]+barOffset);
  Vector3 b4(pos[0]-barLength,pos[1]+barWidth,pos[2]+barOffset);

  Vector3 c1(pos[0]+barWidth,pos[1]-barLength,pos[2]+barOffset);
  Vector3 c2(pos[0]+barWidth,pos[1]+barLength,pos[2]+barOffset);
  Vector3 c3(pos[0]-barWidth,pos[1]-barLength,pos[2]+barOffset);
  Vector3 c4(pos[0]-barWidth,pos[1]+barLength,pos[2]+barOffset);

  white.setCurrentGL();
  glBegin(GL_TRIANGLES);
  glNormal3v(ez);
  glVertex3v(b1);
  glVertex3v(b2);
  glVertex3v(b3);

  glNormal3v(ez);
  glVertex3v(b2);
  glVertex3v(b3);
  glVertex3v(b4);

  glNormal3v(ez);
  glVertex3v(c1);
  glVertex3v(c2);
  glVertex3v(c3);

  glNormal3v(ez);
  glVertex3v(c2);
  glVertex3v(c3);
  glVertex3v(c4);
  glEnd();

  cCross.setCurrentGL();
  glPushMatrix();
  glTranslatef(pos[0], pos[1], pos[2]);
  drawCylinder(cylinderHeight*ez, radius);

  glLineWidth(widthBorder);
  black.setCurrentGL();
  drawWireCircle(ez, radius);

  glPopMatrix();
}

std::vector<double> PathPiecewiseLinear::GetHighCurvatureConfigurations() 
{
    std::vector<double> pathPts;

    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
    ob::SpaceInformationPtr si = gpath.getSpaceInformation();
    std::vector<ob::State *> states = gpath.getStates();
    // double L = GetLength();
    // this->DrawGL(state, 0.5*L);
    // std::cout << "High Curvature Configs" << std::endl;
    double s = 0;
    if (states.size() > 2)
    {
        double a = si->distance(states[0], states[1]);
        // bool lastStatesWasAdded = false;
        for (unsigned int i = 2; i < states.size(); ++i)
        {
            s += a;
            // view the path as a sequence of segments, and look at the triangles it forms:
            //          s1
            //          /\          s4
            //      a  /  \ b       |
            //        /    \        |
            //       /......\_______|
            //     s0    c   s2     s3
            //
            // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
            double b = si->distance(states[i - 1], states[i]);
            double c = si->distance(states[i - 2], states[i]);
            double acosValue = (a * a + b * b - c * c) / (2.0 * a * b);

            if (acosValue > -1.0 && acosValue < 1.0)
            {
                // the smoothness is actually the outside angle of the one we compute
                double angle = (boost::math::constants::pi<double>() - acos(acosValue));

                // and we normalize by the length of the segments
                double k = 2.0 * angle / (a + b);
                double smoothness = k*k;

                // std::cout << s << ":" << smoothness;
                if(smoothness > 1e-3){
                    // std::cout << "*" << std::endl;
                    pathPts.push_back(s);
                }else{
                    // lastStatesWasAdded = false;
                    // std::cout << "" << std::endl;
                }
            }
            a = b;
        }
        if(pathPts.size() <= 0)
        {
            //always have at least one configuration midway displayed
            pathPts.push_back(0.5*length);
        }

    }else{
        //no extremal values, just take midpoint
        pathPts.push_back(0.5*length);
    }
    return pathPts;
}

void PathPiecewiseLinear::DrawGLPathPtr(GUIState& state, ob::PathPtr _path)
{
  std::vector<ob::State *> states;
  if(quotient_space->isDynamic()){
    oc::PathControl *cpath = static_cast<oc::PathControl*>(_path.get());
    states = cpath->getStates();
  }else{
    og::PathGeometric *gpath = static_cast<og::PathGeometric*>(_path.get());
    states = gpath->getStates();
  }
  ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();
  if(states.size() < 2){
    return;
  }

  ob::StateSpacePtr space = si->getStateSpace();

  //############################################################################
  //Set openGL scene
  //############################################################################
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glEnable(GL_LINE_SMOOTH);
  glDisable(GL_CULL_FACE);
  cLine.setCurrentGL();
  //############################################################################

  DrawGLRibbon(states);

  if(drawSweptVolume && state("draw_path_sweptvolume")){
    double L = GetLength();
    this->DrawGL(state, 0.5*L);
    //TODO: make it at high-curvature points
    // std::vector<double> cP = GetHighCurvatureConfigurations();
    // for(uint k = 0; k < cP.size(); k++){
    //     this->DrawGL(state, cP.at(k));
    // }

  }

  //############################################################################
  //Reset openGL
  //############################################################################
  glLineWidth(1);
  glEnable(GL_CULL_FACE);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);

}

CSpaceOMPL* PathPiecewiseLinear::GetSpace() const
{
  return quotient_space;
}
void PathPiecewiseLinear::DrawGL(GUIState& state, double t)
{
  Config q = Eval(t);
  quotient_space->drawConfig(q, cRobotVolume);
  // if(quotient_space->isDynamic()){
  //   Config q = Eval(t);
  //   Config dq = EvalVelocity(t);
  //   quotient_space->drawConfig(q, dq, cRobotVolume);
  // }else{
    // Config q = Eval(t);
    // quotient_space->drawConfig(q, cRobotVolume);
  // }
}

void PathPiecewiseLinear::DrawGL(GUIState& state)
{
  if(quotient_space != nullptr)
  {
    draw_planar = (quotient_space->IsPlanar());
    if(draw_planar && (quotient_space->GetFirstSubspace()->getType()==ob::STATE_SPACE_SE2) && state("planner_draw_spatial_representation_of_SE2")){
      draw_planar = false;
    }
  }

  if(state("draw_path")){
    //if(path==nullptr) return;
    cLine = cSmoothed;
    DrawGLPathPtr(state, path);
  }
  if(state("draw_path_unsmoothed")) 
  {
    //if(path_raw==nullptr) return;
    cLine = cUnsmoothed;
    DrawGLPathPtr(state, path_raw);
  }
}
bool PathPiecewiseLinear::Load(const char* fn)
{
  TiXmlDocument doc(fn);
  std::cout << "Loading from " << fn << std::endl;
  return Load(GetRootNodeFromDocument(doc));
}
bool PathPiecewiseLinear::Load(TiXmlElement *node)
{
  bool res = CheckNodeName(node, "path_piecewise_linear");
  if(!res) return false;

  length = GetSubNodeText<double>(node, "length");

  interLength.clear();

  TiXmlElement* node_il = FindFirstSubNode(node, "interlength");
  while(node_il!=nullptr){
    double tmp;
    GetStreamText(node_il) >> tmp;
    interLength.push_back(tmp);
    node_il = FindNextSiblingNode(node_il);
  }

  if(quotient_space->isDynamic()){

    //############################################################################
    oc::PathControl *cpath = static_cast<oc::PathControl*>(path.get());
    oc::SpaceInformationPtr siC = 
      dynamic_pointer_cast<oc::SpaceInformation>(cpath->getSpaceInformation());
    ob::StateSpacePtr space = siC->getStateSpace();
    cpath->clear();

    std::vector<ob::State*> states;
    TiXmlElement* node_state = FindFirstSubNode(node, "state");
    while(node_state!=nullptr){
      std::vector<double> tmp = GetNodeVector<double>(node_state);
      ob::State *state = siC->allocState();
      space->copyFromReals(state, tmp);
      states.push_back(state);
      node_state = FindNextSiblingNode(node_state);
    }

    //############################################################################
    uint N = quotient_space->GetControlDimensionality();
    std::vector<oc::Control*> controls;
    TiXmlElement* node_ctrl = FindFirstSubNode(node, "control");
    while(node_ctrl!=nullptr){
      std::vector<double> tmp = GetNodeVector<double>(node_ctrl);
      oc::RealVectorControlSpace::ControlType *control = 
        static_cast<oc::RealVectorControlSpace::ControlType*>(siC->allocControl());
      for(uint j = 0; j < N; j++){
        control->values[j] = tmp.at(j);
      }
      controls.push_back(control);
      node_ctrl = FindNextSiblingNode(node_ctrl);
    }
    //############################################################################
    TiXmlElement* node_ctrl_duration = FindFirstSubNode(node, "controlDuration");
    std::vector<double> controlDurations;
    while(node_ctrl_duration!=nullptr){
      std::stringstream ss = GetStreamText(node_ctrl_duration);
      double _tmp;
      ss >> _tmp;
      controlDurations.push_back(_tmp);
      node_ctrl_duration = FindNextSiblingNode(node_ctrl_duration);
    }
    //############################################################################
    for(uint k = 0; k < controls.size(); k++){
      cpath->append(states.at(k), controls.at(k), controlDurations.at(k));
    }
    cpath->append(states.back());
    //############################################################################
  }else{
    {
      og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
      ob::SpaceInformationPtr si = gpath.getSpaceInformation();
      ob::StateSpacePtr space = si->getStateSpace();
      gpath.clear();
      TiXmlElement* node_state = FindFirstSubNode(node, "state");
      while(node_state!=nullptr){
        std::vector<double> tmp = GetNodeVector<double>(node_state);
        ob::State *state = si->allocState();
        space->copyFromReals(state, tmp);
        gpath.append(state);
        node_state = FindNextSiblingNode(node_state);
      }
      path = std::make_shared<og::PathGeometric>(gpath);
    }
    {
      og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path_raw);
      ob::SpaceInformationPtr si = gpath.getSpaceInformation();
      ob::StateSpacePtr space = si->getStateSpace();
      gpath.clear();
      TiXmlElement* node_state = FindFirstSubNode(node, "rawstate");
      while(node_state!=nullptr){
        std::vector<double> tmp = GetNodeVector<double>(node_state);
        ob::State *state = si->allocState();
        space->copyFromReals(state, tmp);
        gpath.append(state);
        node_state = FindNextSiblingNode(node_state);
      }
      path_raw = std::make_shared<og::PathGeometric>(gpath);
    }
  }
  return true;
}

bool PathPiecewiseLinear::Save(const char* fn)
{
  TiXmlDocument doc;
  TiXmlElement *node = CreateRootNodeInDocument(doc);
  Save(node);
  doc.LinkEndChild(node);
  doc.SaveFile(fn);
  return true;
}

int PathPiecewiseLinear::GetNumberOfMilestones()
{
  return interLength.size()+1;
}

bool PathPiecewiseLinear::Save(TiXmlElement *node)
{
  node->SetValue("path_piecewise_linear");
  AddSubNode(*node, "length", length);
  if(!path) return true; //saving zero length path is ok

  AddSubNode(*node, "number_of_milestones", interLength.size()+1);
  AddComment(*node, "Interlength: Length between States");
  for(uint k = 0; k < interLength.size(); k++){
    AddSubNode(*node, "interlength", interLength.at(k));
  }

  if(quotient_space->isDynamic()){
    AddComment(*node, "States: Sequence of Configurations in Bundle Space");

    oc::PathControl *cpath = static_cast<oc::PathControl*>(path.get());

    ob::SpaceInformationPtr si = cpath->getSpaceInformation();
    ob::StateSpacePtr space = si->getStateSpace();

    //############################################################################
    std::vector<ob::State *> states = cpath->getStates();
    for(uint k = 0; k < states.size(); k++){
      std::vector<double> state_k_serialized;
      space->copyToReals(state_k_serialized, states.at(k));
      AddSubNodeVector(*node, "state", state_k_serialized);
    }
    //############################################################################
    uint N = quotient_space->GetControlDimensionality();
    AddComment(*node, "Controls: Sequence of Controls applied inbetwen States");

    std::vector<oc::Control*> &controls = cpath->getControls();
    for(uint k = 0; k < controls.size(); k++){
      double *control = 
        controls.at(k)->as<oc::RealVectorControlSpace::ControlType>()->values;
      std::vector<double> control_k_serialized;
      for(uint j = 0; j < N; j++){
        control_k_serialized.push_back(control[j]);
      }
      AddSubNodeVector(*node, "control", control_k_serialized);
    }
    //############################################################################
    AddComment(*node, "Duration for each Control");
    std::vector<double> ctrlDurations = cpath->getControlDurations();

    for(uint k = 0; k < ctrlDurations.size(); k++){
      AddSubNode(*node, "controlDuration", ctrlDurations.at(k));
    }


  }else{
    {
      AddComment(*node, "Smoothed States: Sequence of Configurations in Configuration Space");

      og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
      ob::SpaceInformationPtr si = gpath.getSpaceInformation();
      ob::StateSpacePtr space = si->getStateSpace();
      std::vector<ob::State *> states = gpath.getStates();
      for(uint k = 0; k < states.size(); k++){
        std::vector<double> state_k_serialized;
        space->copyToReals(state_k_serialized, states.at(k));
        AddSubNodeVector(*node, "state", state_k_serialized);
      }
    }

    {
      AddComment(*node, "Raw States: Unsmoothed");

      og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path_raw);
      ob::SpaceInformationPtr si = gpath.getSpaceInformation();
      ob::StateSpacePtr space = si->getStateSpace();
      std::vector<ob::State *> states = gpath.getStates();
      for(uint k = 0; k < states.size(); k++){
        std::vector<double> state_k_serialized;
        space->copyToReals(state_k_serialized, states.at(k));
        AddSubNodeVector(*node, "rawstate", state_k_serialized);
      }
    }
  }
  return true;
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
  return out;
}
