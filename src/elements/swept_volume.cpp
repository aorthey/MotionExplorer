#include <tinyxml.h>
#include "swept_volume.h"
#include "util.h"

SweptVolume::SweptVolume(Robot *robot){
  _keyframes.clear();
  _mats.clear();
  _robot = robot;
  color = GLColor(0.8,0.8,0.8);
  uint Nlinks = _robot->links.size();

  _appearanceStack.clear();
  _appearanceStack.resize(Nlinks);

  for(size_t i=0;i<Nlinks;i++) {
    GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
    _appearanceStack[i]=a;
  }
}

SweptVolume::SweptVolume(Robot *robot, const std::vector<Config> &keyframes, uint Nkeyframes)
{
  _keyframes.clear();
  _mats.clear();
  _robot = robot;
  color = GLColor(0.8,0.8,0.8);
  uint Nlinks = _robot->links.size();

  _appearanceStack.clear();
  _appearanceStack.resize(Nlinks);

  for(size_t i=0;i<Nlinks;i++) {
    GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
    _appearanceStack[i]=a;
  }

  for(int i = 0; i < keyframes.size(); i++)
  {
    Config q = keyframes.at(i);
    AddKeyframe(q);
  }
  if(keyframes.size()>0){
    init = _keyframes.front();
    goal = _keyframes.back();
  }
  //compute keyframe indices 
  if(Nkeyframes > 0){
    if(Nkeyframes> keyframes.size()){
      Nkeyframes = keyframes.size();
    }
    _keyframe_indices.clear();

    uint N = keyframes.size();
    uint Nstep = (int)(N/Nkeyframes);

    if(Nstep<1) Nstep=1;
    uint Ncur = 0;
    while(Ncur < N){
      _keyframe_indices.push_back(Ncur);
      Ncur += Nstep;
    }
  }
}

Robot* SweptVolume::GetRobot(){
  return _robot;
}
const std::vector<std::vector<Matrix4> >& SweptVolume::GetMatrices(){
  return _mats;
}
const std::vector<Config >& SweptVolume::GetKeyframes(){
  return _keyframes;
}
const vector<GLDraw::GeometryAppearance>& SweptVolume::GetAppearanceStack(){
  return _appearanceStack;
}

void SweptVolume::SetColor(const GLColor c){
  color = c;
}
GLColor SweptVolume::GetColor(){
  return color;
}
void SweptVolume::SetColorMilestones(const GLColor c){
  color_milestones= c;
}
GLColor SweptVolume::GetColorMilestones(){
  return color_milestones;
}
const Config& SweptVolume::GetStart(){
  return init;
}
const Config& SweptVolume::GetGoal(){
  return goal;
}
const vector<uint>& SweptVolume::GetKeyframeIndices(){
  return _keyframe_indices;
}
void SweptVolume::AddKeyframe(const Config &q ){

  if(!_robot->InJointLimits(q)){
    std::cout << "trying to set an outer limit config" << std::endl;
    std::cout << "minimum       :" << _robot->qMin << std::endl;
    std::cout << "configuration :" << q << std::endl;
    std::cout << "maximum       :" << _robot->qMax << std::endl;
    //check joint limits
    for(int i = 0; i < _robot->q.size(); i++){
      if(q(i) < _robot->qMin(i) || q(i) > _robot->qMax(i)){
        std::cout << "[ "<<i<< " ]: " << _robot->qMin(i) << " < " << q(i) << " < " << _robot->qMax(i) << std::endl;
      }
    }

    exit(0);
  }
  _robot->UpdateConfig(q);

  std::vector<Matrix4> mats_config;
  uint Nlinks = _robot->links.size();

  for(size_t i=0;i<Nlinks;i++) {
    Matrix4 mat = _robot->links[i].T_World;
    mats_config.push_back(mat);
  }
  _mats.push_back(mats_config);
  _keyframes.push_back(q);
}


bool SweptVolume::Load(const char* file)
{
  std::string pdata = util::GetDataFolder();
  std::string in = pdata+"/sweptvolume/"+file;

  std::cout << "loading data from "<<in << std::endl;

  TiXmlDocument doc(in.c_str());
  if(doc.LoadFile()){
    TiXmlElement *root = doc.RootElement();
    if(root){
      Load(root);
    }
  }else{
    std::cout << doc.ErrorDesc() << std::endl;
    std::cout << "ERROR" << std::endl;
    exit(0);
  }
  return true;
}
bool SweptVolume::Save(const char* file)
{
  std::string out;
  std::string pdata = util::GetDataFolder();

  if(!file){
    std::string date = util::GetCurrentTimeString();
    out = pdata+"/sweptvolume/state_"+date+".xml";
  }else{
    out = pdata+"/sweptvolume/"+file;
  }

  std::cout << "saving data to "<< out << std::endl;

  TiXmlDocument doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  doc.LinkEndChild(decl);
  TiXmlElement *node = new TiXmlElement("SweptVolume");
  Save(node);

  doc.LinkEndChild(node);
  doc.SaveFile(out.c_str());

  return true;

}
bool SweptVolume::Save(TiXmlElement *node)
{
  node->SetValue("SweptVolume");

  //###################################################################
  {
    TiXmlElement c("keyframes");
    for(int i = 0; i < _keyframes.size(); i++){
      TiXmlElement cc("qitem");
      stringstream ss;
      ss<<_keyframes.at(i);
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    node->InsertEndChild(c);
  }
  //###################################################################
  {
    TiXmlElement c("matrices");
    for(int i = 0; i < _mats.size(); i++){
      TiXmlElement cc("matrixvector");
      for(int j = 0; j < _mats.at(i).size(); j++){
        TiXmlElement ccc("matrix");
        stringstream ss;
        ss<<_mats.at(i).at(j);
        TiXmlText text(ss.str().c_str());
        ccc.InsertEndChild(text);
        cc.InsertEndChild(ccc);
      }
      c.InsertEndChild(cc);
    }
    node->InsertEndChild(c);
  }
  //###################################################################
  {
    TiXmlElement c("keyframe_indices");
    for(int i = 0; i < _keyframe_indices.size(); i++){
      TiXmlElement cc("index");
      stringstream ss;
      ss<<_keyframe_indices.at(i);
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    node->InsertEndChild(c);
  }
  //###################################################################
  {
    TiXmlElement c("color");
    stringstream ss;
    ss<<color;
    TiXmlText text(ss.str().c_str());
    node->InsertEndChild(text);
  }
  //###################################################################
  {
    TiXmlElement c("color_milestones");
    stringstream ss;
    ss<<color_milestones;
    TiXmlText text(ss.str().c_str());
    node->InsertEndChild(text);
  }

  init = _keyframes.front();
  goal = _keyframes.back();

  return true;

}

bool SweptVolume::Load(TiXmlElement *node)
{
    //GLColor color;
    //GLColor color_milestones;

    //Robot *_robot;
    //std::vector<std::vector<Matrix4> > _mats;
    //vector<Config> _keyframes;
    //Config init, goal;
    //vector<uint> _keyframe_indices;

  _mats.clear();
  _keyframes.clear();
  _keyframe_indices.clear();

  if(0!=strcmp(node->Value(),"SweptVolume")) {
    std::cout << "Not a SweptVolume file" << std::endl;
    return false;
  }
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) 
  {
    if(0==strcmp(e->Value(),"keyframes")) {

      TiXmlElement* c=e->FirstChildElement();
      while(c!=NULL)
      {
        if(0==strcmp(c->Value(),"qitem")) {
          Config q;
          stringstream ss(c->GetText());
          ss >> q;
          _keyframes.push_back(q);
        }
        c = c->NextSiblingElement();
      }
    }
    if(0==strcmp(e->Value(),"matrices")) {

      TiXmlElement* c=e->FirstChildElement();
      while(c!=NULL)
      {
        if(0==strcmp(c->Value(),"matrixvector")) {
          TiXmlElement* cc=c->FirstChildElement();
          std::vector<Matrix4> matvec;
          while(cc!=NULL)
          {
            if(0==strcmp(cc->Value(),"matrix")) {
              Matrix4 m;
              stringstream ss(cc->GetText());
              ss >> m;
              matvec.push_back(m);
            }
            cc = cc->NextSiblingElement();
          }
          _mats.push_back(matvec);
        }
        c = c->NextSiblingElement();
      }
    }
    if(0==strcmp(e->Value(),"keyframe_indices")) {

      TiXmlElement* c=e->FirstChildElement();
      while(c!=NULL)
      {
        if(0==strcmp(c->Value(),"index")) {
          uint k;
          stringstream ss(c->GetText());
          ss >> k;
          _keyframe_indices.push_back(k);
        }
        c = c->NextSiblingElement();
      }
    }
    e = e->NextSiblingElement();
  }

  return true;
}
