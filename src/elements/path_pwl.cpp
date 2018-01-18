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

PathPiecewiseLinear::PathPiecewiseLinear(CSpaceOMPL *cspace_):
  cspace(cspace_)
{
}

PathPiecewiseLinear::PathPiecewiseLinear(ob::PathPtr p_, CSpaceOMPL *cspace_):
  cspace(cspace_), path(p_), path_raw(p_)
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
  if(!path) return;

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
Config PathPiecewiseLinear::Eval(const double t) const{
  if(!path){
    std::cout << "Cannot Eval empty path" << std::endl;
    exit(0);
  }
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
  ob::SpaceInformationPtr si = gpath.getSpaceInformation();
  std::vector<ob::State *> states = gpath.getStates();

  if(t<=0) return cspace->OMPLStateToConfig(states.front());
  if(t>=length) return cspace->OMPLStateToConfig(states.back());

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
      Config qm = cspace->OMPLStateToConfig(sm);
      si->freeState(sm);
      return qm;
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
void PathPiecewiseLinear::DrawGL(GUIState& state)
{
  if(!path) return;
  cLine = magenta;
  DrawGLPathPtr(path);
  if(!path_raw) return;
  cLine = green;
  DrawGLPathPtr(path_raw);
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

  //void  copyToReals (std::vector< double > &reals, const State *source) const
      //Copy all the real values from a state source to the array reals using getValueAddressAtLocation()
       
      //void  copyFromReals (State *destination, const std::vector< double > &reals) const
        //Copy the values from reals to the state destination using getValueAddressAtLocation()
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
