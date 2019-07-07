#include "elements/path_pwl.h"
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "gui/drawMotionPlanner.h"
#include <iostream>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/control/PathControl.h>

PathPiecewiseLinear::PathPiecewiseLinear(CSpaceOMPL *cspace_):
  cspace(cspace_)
{
}

PathPiecewiseLinear::PathPiecewiseLinear(ob::PathPtr p_, CSpaceOMPL *cspace_, CSpaceOMPL *quotient_space_):
  cspace(cspace_), quotient_space(quotient_space_), path(p_), path_raw(p_)
{
  if(1){//!cspace->isDynamic()){
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);

    length = gpath.length();
    std::vector<ob::State *> states = gpath.getStates();

    for(uint k = 0; k < states.size()-1; k++){
      ob::State *s0 = states.at(k);
      ob::State *s1 = states.at(k+1);
      interLength.push_back(gpath.getSpaceInformation()->distance(s0,s1));
    }

    path = std::make_shared<og::PathGeometric>(gpath);
  }else{
    oc::PathControl cpath = static_cast<oc::PathControl&>(*path);

    length = cpath.length();
    std::vector<ob::State *> states = cpath.getStates();

    for(uint k = 0; k < states.size()-1; k++){
      ob::State *s0 = states.at(k);
      ob::State *s1 = states.at(k+1);
      interLength.push_back(cpath.getSpaceInformation()->distance(s0,s1));
    }

    path = std::make_shared<oc::PathControl>(cpath);
  }
}

void PathPiecewiseLinear::SendToController(SmartPointer<RobotController> controller)
{
  if(!cspace->isDynamic()){
    std::cout << "Path is not dynamic, cannot access torques" << std::endl;
    return;
  }

  std::vector<string> cmds = controller->Commands();
  for(uint k = 0; k < cmds.size(); k++){
    std::cout << cmds.at(k) << std::endl;
  }
  oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
  
  std::vector<oc::Control*> controls = cpath.getControls();

  uint N = quotient_space->GetControlDimensionality();
  for(uint k = 0; k < controls.size(); k++){
    const oc::RealVectorControlSpace::ControlType *Rctrl = controls.at(k)->as<oc::RealVectorControlSpace::ControlType>();
    stringstream qstr;
    qstr << N << "  ";
    for(uint i = 0; i < N; i++){
      qstr<< Rctrl->values[i] << " ";
    }
    string cmd( (k<=0)?("set_torque_control"):("append_torque_control") );
    if(!controller->SendCommand(cmd,qstr.str())) {
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "ERROR in controller commander" << std::endl;
      std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      throw "Controller command not supported!";
    }
  }

}
// -void MotionPlanner::SendCommandStringController(string cmd, string arg)
// -{
// -  if(!_sim->robotControllers[_icontroller]->SendCommand(cmd,arg)) {
// -    std::cout << std::string(80, '-') << std::endl;
// -    std::cout << "ERROR in controller commander" << std::endl;
// -    std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
// -    std::cout << std::string(80, '-') << std::endl;
// -    throw "Controller command not supported!";
 // }
// -bool MotionPlanner::SendToController()
// -{
// -  if(!_isSolved){ return false; }
 
// -  double dstep = 0.1;
// -  Config q;
// -  Config dq;
// -  for(int i = 0; i < _keyframes.size()-1; i++){
// -    //_path.Evaluate(d, q, dq);
// -    q = _keyframes.at(i);
// -    Config q2 = _keyframes.at(i+1);
// -    double dt = 1.0/_keyframes.size();
// -    dq = (q-q2)/dt;
// -    stringstream qstr;
// -    qstr<<q<<dq;
// -    string cmd( (i<=0)?("set_qv"):("append_qv") );
// -    SendCommandStringController(cmd,qstr.str());


void PathPiecewiseLinear::Smooth(){
  if(path == nullptr) return;
  if(!isSmooth){

    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
    std::vector<ob::State *> statesB = gpath.getStates();

    og::PathSimplifier shortcutter(gpath.getSpaceInformation());
    shortcutter.simplifyMax(gpath);
    shortcutter.smoothBSpline(gpath);

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

Vector3 PathPiecewiseLinear::EvalVec3(const double t) const{
  Config q = Eval(t);
  ob::ScopedState<> s = quotient_space->ConfigToOMPLState(q);
  Vector3 v = quotient_space->getXYZ(s.get());
  if(draw_planar) v[2] = 0.0;
  return v;
}

Config PathPiecewiseLinear::EvalMilestone(const int k) const{
  std::cout << "NYI" << std::endl;
  exit(0);
  //if(k<0) return keyframes.front();
  //if(k>Nkeyframes) return keyframes.back();
  //return keyframes.at(k);
}

Config PathPiecewiseLinear::Eval(const double t) const{
  if(!path){
    std::cout << "Cannot Eval empty path" << std::endl;
    exit(0);
  }

  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
  std::vector<ob::State *> states = gpath.getStates();

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
    throw;
  }

  if(t>=Tcum){
    return quotient_space->OMPLStateToConfig(states.back());
  }

  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;

}
Config PathPiecewiseLinear::EvalVelocity(const double t) const{
  if(!quotient_space->isDynamic()) 
  {
    Config dq = quotient_space->GetRobotPtr()->dq;
    dq.setZero();
    return dq;
  }

  KinodynamicCSpaceOMPL *kspace = static_cast<KinodynamicCSpaceOMPL*>(quotient_space);

  oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
  std::vector<ob::State *> states = cpath.getStates();

  ob::SpaceInformationPtr si = quotient_space->SpaceInformationPtr();

  if(t<=0){
    return kspace->OMPLStateToVelocity(states.front());
  }
  if(t>=length){
    return kspace->OMPLStateToVelocity(states.back());
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
      Config q = kspace->OMPLStateToVelocity(sm);
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
    return kspace->OMPLStateToVelocity(states.back());
  }

  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;
}

// void PathPiecewiseLinear::DrawGLPathPtr(ob::PathPtr _path){
//   og::PathGeometric gpath = static_cast<og::PathGeometric&>(*_path);
//   ob::SpaceInformationPtr si = gpath.getSpaceInformation();
//   std::vector<ob::State *> states = gpath.getStates();

//   ob::StateSpacePtr space = si->getStateSpace();

//   glDisable(GL_LIGHTING);
//   glEnable(GL_BLEND);
//   glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
//   glEnable(GL_LINE_SMOOTH);
//   glPushMatrix();

//   glPointSize(ptsize);
//   glLineWidth(linewidth);
//   cLine.setCurrentGL();
//   for(uint i = 0; i < states.size()-1; i++){
//     ob::State* c1 = states.at(i);
//     ob::State* c2 = states.at(i+1);
//     Vector3 q1 = quotient_space->getXYZ(c1);
//     Vector3 q2 = quotient_space->getXYZ(c2);
//     if(draw_planar){
//       q1[2] = 0.0; q2[2] = 0.0;
//     }
//     GLDraw::drawPoint(q1);
//     GLDraw::drawLineSegment(q1, q2);
//   }
//   glPopMatrix();
//   glDisable(GL_BLEND);
//   glEnable(GL_LIGHTING);
//   glLineWidth(1);
// }
Vector3 PathPiecewiseLinear::Vector3FromState(ob::State *s){
  Vector3 v = quotient_space->getXYZ(s);
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
    std::vector<ob::State*> states, uint k_start_state, double arrow_size_length)
{
    uint m = k_start_state;
    Vector3 qnext = Vector3FromState(states.at(m));
    double zmax = qnext[2];
    double d_tip_to_state_best = fabs( arrow_pos.distanceSquared(qnext) - arrow_size_length);

    m = m+1;
    qnext = Vector3FromState(states.at(m));
    if(qnext[2] > zmax) zmax = qnext[2];
    double d_tip_to_state_next = fabs( arrow_pos.distanceSquared(qnext) - arrow_size_length);

    while(d_tip_to_state_next < d_tip_to_state_best){
      d_tip_to_state_best = d_tip_to_state_next;

      m = m+1;
      qnext = Vector3FromState(states.at(m));
      if(qnext[2] > zmax) zmax = qnext[2];
      d_tip_to_state_next = fabs( arrow_pos.distanceSquared(qnext) - arrow_size_length);
    }
    m = m-1;
    qnext = Vector3FromState(states.at(m));
    qnext[2] = zmax;
    return qnext;
}
void PathPiecewiseLinear::DrawGLPathPtr(ob::PathPtr _path){
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*_path);
  ob::SpaceInformationPtr si = gpath.getSpaceInformation();
  std::vector<ob::State *> states = gpath.getStates();

  ob::StateSpacePtr space = si->getStateSpace();

  //############################################################################
  //Set openGL scene
  //############################################################################
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glEnable(GL_LINE_SMOOTH);
  glDisable(GL_CULL_FACE);

  glPushMatrix();
  glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
  glPointSize(ptsize);
  cLine.setCurrentGL();

  //############################################################################
  //Compute Arrow Position
  //############################################################################
  if(states.size() < 2){
    std::cout << "Path has " << states.size() << " states." << std::endl;
    std::cout << "Cannot draw arrow." << std::endl;
    exit(0);
  }

  Vector3 arrow_pos, arrow_dir;
  double arrow_size_head = 2*linewidth;
  double arrow_size_length = 1.5*arrow_size_head;

  glLineWidth(arrow_size_head*10);
  Vector3 qnext;
  if(states.size() == 2){
    Vector3 q1 = Vector3FromState(states.at(0));
    qnext = Vector3FromState(states.at(1));
    arrow_pos = 0.5*(qnext - q1);
  }else if(states.size()%2 == 0){
    uint m = 0.5*states.size();
    Vector3 q1 = Vector3FromState(states.at(m-1));
    Vector3 q2 = Vector3FromState(states.at(m));
    arrow_pos = q1 + 0.5*(q2 - q1);
    qnext = GetNearestStateToTipOfArrow(arrow_pos, states, m, arrow_size_length);
  }else{
    uint m = floor(0.5*states.size());
    arrow_pos = Vector3FromState(states.at(m));
    qnext = GetNearestStateToTipOfArrow(arrow_pos, states, m, arrow_size_length);
  }
  arrow_dir = qnext - arrow_pos;
  arrow_dir.inplaceNormalize();
  arrow_dir.inplaceMul(arrow_size_length);

  //############################################################################
  //Draws a tron-like line strip
  //############################################################################
  glBegin(GL_QUAD_STRIP);
  std::vector<Vector3> path_left;
  std::vector<Vector3> path_right;
  for(uint i = 0; i < states.size(); i++){
    Vector3 q1 = Vector3FromState(states.at(i));
    Vector3 dq;

    if(i<states.size()-1){
      Vector3 q2 = Vector3FromState(states.at(i+1));
      dq = q2 - q1;
    }else{
      Vector3 q2 = Vector3FromState(states.at(i-1));
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
  black.setCurrentGL();
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

  //############################################################################
  //Draw Arrow at middle of path
  //############################################################################
  glPushMatrix();
  glTranslatef(arrow_pos[0], arrow_pos[1], arrow_pos[2]+std::max(0.5*zOffset, 1e-3));
  Draw2DArrow(arrow_pos, arrow_dir, arrow_size_head, arrow_size_length);
  glPopMatrix();
  glEnable(GL_CULL_FACE);

  //############################################################################
  //Reset openGL
  //############################################################################
  glPopMatrix();
  glLineWidth(1);
  glEnable(GL_CULL_FACE);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
  // glDisable(GL_BLEND);
  // glEnable(GL_LIGHTING);

}

void PathPiecewiseLinear::DrawGL(GUIState& state, double t)
{
  Config q = Eval(t);
  Robot* robot = quotient_space->GetRobotPtr();
  GLDraw::drawRobotAtConfig(robot, q, grey);
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
    DrawGLPathPtr(path);
  }
  if(state("draw_path_unsmoothed")) 
  {
    //if(path_raw==nullptr) return;
    cLine = cUnsmoothed;
    DrawGLPathPtr(path_raw);
  }
  if(state("draw_path_sweptvolume")){
    if(!path) return;
    if(!sv){
      double tmin = 0.05;
      double L = GetLength();
      uint Nmilestones = int(L/tmin);
      std::vector<Config> q;
      for(uint k = 0; k < Nmilestones; k++){
        q.push_back( Eval(k*tmin) );
      }
      sv = new SweptVolume(quotient_space->GetRobotPtr(), q, Nmilestones);
    }
    sv->DrawGL(state);
  }
}
bool PathPiecewiseLinear::Load(const char* fn)
{
  TiXmlDocument doc(fn);
  return Load(GetRootNodeFromDocument(doc));
}
bool PathPiecewiseLinear::Load(TiXmlElement *node)
{
  bool res = CheckNodeName(node, "path_piecewise_linear");
  if(!res) return false;

  length = GetSubNodeText<double>(node, "length");

  interLength.clear();

  TiXmlElement* node_il = FindFirstSubNode(node, "interlength");
  while(node_il!=NULL){
    double tmp;
    GetStreamText(node_il) >> tmp;
    interLength.push_back(tmp);
    node_il = FindNextSiblingNode(node_il);
  }

  {
    og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
    ob::SpaceInformationPtr si = gpath.getSpaceInformation();
    ob::StateSpacePtr space = si->getStateSpace();
    gpath.clear();
    TiXmlElement* node_state = FindFirstSubNode(node, "state");
    while(node_state!=NULL){
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
    while(node_state!=NULL){
      std::vector<double> tmp = GetNodeVector<double>(node_state);
      ob::State *state = si->allocState();
      space->copyFromReals(state, tmp);
      gpath.append(state);
      node_state = FindNextSiblingNode(node_state);
    }
    path_raw = std::make_shared<og::PathGeometric>(gpath);
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
