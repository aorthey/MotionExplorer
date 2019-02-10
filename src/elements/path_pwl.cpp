#include "elements/path_pwl.h"
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_kinodynamic.h"
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

  uint N = cspace->GetControlDimensionality();
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
  ob::ScopedState<> s = cspace->ConfigToOMPLState(q);
  Vector3 v = cspace->getXYZ(s.get());
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

  ob::SpaceInformationPtr si = cspace->SpaceInformationPtr();

  if(t<=0){
    return cspace->OMPLStateToConfig(states.front());
  }
  if(t>=length){
    return cspace->OMPLStateToConfig(states.back());
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
      Config q = cspace->OMPLStateToConfig(sm);
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
    return cspace->OMPLStateToConfig(states.back());
  }

  std::cout << "Eval could not find point for value " << t << std::endl;
  throw;

}
Config PathPiecewiseLinear::EvalVelocity(const double t) const{
  if(!cspace->isDynamic()) 
  {
    Config dq = cspace->GetRobotPtr()->dq;
    dq.setZero();
    return dq;
  }

  KinodynamicCSpaceOMPL *kspace = static_cast<KinodynamicCSpaceOMPL*>(cspace);

  oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
  std::vector<ob::State *> states = cpath.getStates();

  ob::SpaceInformationPtr si = cspace->SpaceInformationPtr();

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
  // if(!path){
  //   std::cout << "Cannot Eval empty path" << std::endl;
  //   exit(0);
  // }
  // og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
  // ob::SpaceInformationPtr si = cspace->SpaceInformationPtr();
  // std::vector<ob::State *> states = gpath.getStates();

  // if(t<=0) return cspace->OMPLStateToConfig(states.front());
  // if(t>=length) return cspace->OMPLStateToConfig(states.back());

  // double Tcum = 0;

  // assert(interLength.size()==states.size()-1);

  // for(uint i = 0; i < interLength.size(); i++){
  //   double Tnext = interLength.at(i);
  //   if((Tcum+Tnext)>=t){
  //     //t \in [Lcum, Lcum+Lnext]
  //     double tloc = (t-Tcum)/Tnext; //tloc \in [0,1]

  //     ob::State* s1 = states.at(i);
  //     ob::State* s2 = states.at(i+1);
  //     ob::State* sm = si->allocState();
  //     si->getStateSpace()->interpolate(s1,s2,tloc,sm);
  //     Config qm = cspace->OMPLStateToConfig(sm);
  //     si->freeState(sm);
  //     return qm;
  //   }
  //   Tcum+=Tnext;
  // }
  // //rounding errors could lead to the fact that the cumulative length is not
  // //exactly 1. If t is sufficiently close, we just return the last keyframe.
  // double epsilon = 1e-10;
  // if(length-Tcum > epsilon){
  //   std::cout << "length of path is significantly different from " << length << std::endl;
  //   std::cout << "length    : " << Tcum << "/" << length << std::endl;
  //   std::cout << "difference: " << length-Tcum << " > " << epsilon << std::endl;
  //   std::cout << "choosen t : " << t << std::endl;
  //   throw;
  // }

  // if(t>=Tcum){
  //   return cspace->OMPLStateToConfig(states.back());
  // }

  // std::cout << "Eval could not find point for value " << t << std::endl;
  // throw;

void PathPiecewiseLinear::DrawGLPathPtr(ob::PathPtr _path){
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*_path);
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
    Vector3 q1 = cspace->getXYZ(c1);
    Vector3 q2 = cspace->getXYZ(c2);
    GLDraw::drawPoint(q1);
    GLDraw::drawLineSegment(q1, q2);
  }
  glPopMatrix();
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
  glLineWidth(1);
}

void PathPiecewiseLinear::DrawGL(GUIState& state, double t)
{
  Config q = Eval(t);
  Robot* robot = quotient_space->GetRobotPtr();
  GLDraw::drawRobotAtConfig(robot, q, grey);
}

void PathPiecewiseLinear::DrawGL(GUIState& state)
{
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
      sv = new SweptVolume(cspace->GetRobotPtr(), q, Nmilestones);
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
