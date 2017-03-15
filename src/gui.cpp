#include <tinyxml.h>
#include <iostream>
#include <fstream>
#include "gui.h"
#include "drawMotionPlanner.h"
#include "util.h"


const GLColor bodyColor(0.1,0.1,0.1);
const GLColor selectedLinkColor(1.0,1.0,0.5);
const double sweptvolumeScale = 0.98;
const double sweptVolume_q_spacing = 0.01;

ForceFieldBackend::ForceFieldBackend(RobotWorld *world)
    : SimTestBackend(world)
{
  drawForceField = 0;
  drawRobotExtras = 1;
  drawIKextras = 0;
  drawPath = 1;
  drawPlannerTree = 1;
  drawPlannerStartGoal = 1;
  drawRigidObjects = 1;

  drawAxes = 1;
  drawAxesLabels = 0;

  MapButtonToggle("draw_planner_tree",&drawPlannerTree);
  MapButtonToggle("draw_path",&drawPath);
  MapButtonToggle("draw_path_start_goal",&drawPlannerStartGoal);
  MapButtonToggle("draw_rigid_objects",&drawRigidObjects);
  MapButtonToggle("draw_fancy_coordinate_axes",&drawAxes);
  MapButtonToggle("draw_fancy_coordinate_axes_labels",&drawAxesLabels);


  _mats.clear();
}


void ForceFieldBackend::Start()
{
  BaseT::Start();


  //disable higher drawing functions
  //drawBBs,drawPoser,drawDesired,drawEstimated,drawContacts,drawWrenches,drawExpanded,drawTime,doLogging
  drawPoser = 0;

  //settings["desired"]["color"][0] = 1;
  //settings["desired"]["color"][1] = 0;
  //settings["desired"]["color"][2] = 0;
  //settings["desired"]["color"][3] = 0.5;
  //camera.dist = 4;
  //viewport.n = 0.1;
  //viewport.f = 100;
  //viewport.setLensAngle(DtoR(90.0));
  //DisplayCameraTarget();
  //Camera::CameraController_Orbit camera;
  //Camera::Viewport viewport;
  show_frames_per_second = true;

}

//############################################################################
//############################################################################


void ForceFieldBackend::RenderWorld()
{
  DEBUG_GL_ERRORS()
  drawDesired=0;

  for(size_t i=0;i<world->terrains.size();i++)
    world->terrains[i]->DrawGL();

  if(drawRigidObjects){
    for(size_t i=0;i<world->rigidObjects.size();i++){
      //
      //visualization of vertices/faces is implemented in 
      //Krislibrary/GLDraw/GeometryAppearances.h
      //but edges are not yet!
      //

      RigidObject *obj = world->rigidObjects[i];
      GLDraw::GeometryAppearance* a = obj->geometry.Appearance();
      a->SetColor(GLColor(0.8,0.8,0.8));
      a->drawFaces = true;
      //a->drawEdges = true;
      //a->drawVertices = true;
      //a->vertexSize = 10;
      //a->edgeSize = 500;
      obj->DrawGL();
    }
  }
  //WorldGUIBackend::RenderWorld();

  for(size_t i=0;i<world->robots.size();i++) {
    for(size_t j=0;j<world->robots[i]->links.size();j++) {
      sim.odesim.robot(i)->GetLinkTransform(j,world->robots[i]->links[j].T_World);
      world->robotViews[i].DrawLink_World(j);
    }
  }


  BaseT::RenderWorld();

  glEnable(GL_BLEND);
  allWidgets.Enable(&allRobotWidgets,drawPoser==1);
  allWidgets.DrawGL(viewport);
  vector<ViewRobot> viewRobots = world->robotViews;

  Robot *robot = world->robots[0];
  ViewRobot *viewRobot = &viewRobots[0];

  //############################################################################
  // Visualize
  //
  // drawrobotextras      : COM, skeleton
  // drawikextras         : contact links, contact directions
  // drawforcefield       : a flow/force field on R^3
  // drawpath             : swept volume along path
  // drawplannerstartgoal : start/goal configuration of motion planner
  // drawplannertree      : Cspace tree visualized as COM tree in W
  // drawaxes             : fancy coordinate axes
  // drawaxeslabels       : labelling of the coordinate axes [needs fixing]
  //############################################################################

  if(drawRobotExtras) GLDraw::drawRobotExtras(viewRobot);
  if(drawIKextras) GLDraw::drawIKextras(viewRobot, robot, _constraints, _linksInCollision, selectedLinkColor);
  if(drawForceField) GLDraw::drawUniformForceField();
  if(drawPath) GLDraw::drawPathSweptVolume(robot, _mats, _appearanceStack);
  if(drawPlannerStartGoal) GLDraw::drawPlannerStartGoal(robot, planner_p_init, planner_p_goal);
  if(drawPlannerTree) GLDraw::drawPlannerTree(_stree);
  if(drawAxes) drawCoordWidget(1); //void drawCoordWidget(float len,float axisWidth=0.05,float arrowLen=0.2,float arrowWidth=0.1);
  if(drawAxesLabels) GLDraw::drawAxesLabels(viewport);

  

}//RenderWorld

bool ForceFieldBackend::Load(TiXmlElement *node)
{

  _stree.clear();
  _keyframes.clear();
  _mats.clear();
  _appearanceStack.clear();
  planner_p_goal.setZero();
  planner_p_init.setZero();

  if(0!=strcmp(node->Value(),"GUI")) {
    std::cout << "Not a GUI file" << std::endl;
    return false;
  }
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) 
  {
    if(0==strcmp(e->Value(),"robot")) {
      TiXmlElement* c=e->FirstChildElement();
      while(c!=NULL)
      {
        if(0==strcmp(c->Value(),"name")) {
          if(e->GetText()){
            stringstream ss(e->GetText());
            ss >> _robotname;
          }
        }
        if(0==strcmp(c->Value(),"configinit")) {
          stringstream ss(c->GetText());
          ss >> planner_p_init;
        }
        if(0==strcmp(c->Value(),"configgoal")) {
          stringstream ss(c->GetText());
          ss >> planner_p_goal;
        }
        c = c->NextSiblingElement();
      }
    }
    if(0==strcmp(e->Value(),"tree")) {
      TiXmlElement* c=e->FirstChildElement();
      while(c!=NULL)
      {
        if(0==strcmp(c->Value(),"node")) {
          SerializedTreeNode sn;
          sn.Load(c);
          _stree.push_back(sn);
        }
        c = c->NextSiblingElement();
      }
    }
    if(0==strcmp(e->Value(),"sweptvolume")) {

      TiXmlElement* c=e->FirstChildElement();
      std::vector<Config> keyframes;
      while(c!=NULL)
      {
        if(0==strcmp(c->Value(),"qitem")) {
          Config q;
          stringstream ss(c->GetText());
          ss >> q;
          keyframes.push_back(q);
        }
        c = c->NextSiblingElement();
      }
      VisualizePathSweptVolume(keyframes);
    }
    e = e->NextSiblingElement();
  }

  return true;
}
bool ForceFieldBackend::Load(const char* file)
{
  std::string pdata = util::GetDataFolder();
  std::string in = pdata+"/gui/"+file;
  //std::string in = "../gui/state_2017_03_14.xml";

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
  }
  //exit(0);
  return true;

}
bool ForceFieldBackend::Save()
{

  std::string date = util::GetCurrentTimeString();
  std::string pdata = util::GetDataFolder();
  std::string out = pdata+"/gui/state_"+date+".xml";

  std::cout << "saving data to "<<out << std::endl;

  TiXmlDocument doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  doc.LinkEndChild(decl);
  TiXmlElement *node = new TiXmlElement("GUI");
  Save(node);

  doc.LinkEndChild(node);
  doc.SaveFile(out.c_str());

  return true;

}
bool ForceFieldBackend::Save(TiXmlElement *node)
{
  node->SetValue("GUI");

  //###################################################################
  {
    TiXmlElement c("robot");

    {
      TiXmlElement cc("name");
      stringstream ss;
      ss<<_robotname;
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    {
      TiXmlElement cc("configinit");
      stringstream ss;
      ss<<planner_p_init<<endl;
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    {
      TiXmlElement cc("configgoal");
      stringstream ss;
      ss<<planner_p_goal;
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    node->InsertEndChild(c);
  }
  //###################################################################
  {
    TiXmlElement c("tree");
    for(int i = 0; i < _stree.size(); i++){
      TiXmlElement cc("node");
      _stree.at(i).Save(&cc);
      c.InsertEndChild(cc);
    }

    node->InsertEndChild(c);
  }
  //###################################################################
  {
    TiXmlElement c("sweptvolume");
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

  return true;
   // std::vector<std::vector<Matrix4> > _mats;
   // vector<GLDraw::GeometryAppearance> _appearanceStack;

}

//############################################################################
//############################################################################

#include <KrisLibrary/graph/Tree.h>

void ForceFieldBackend::VisualizePlannerTree(const SerializedTree &tree)
{
//typedef std::pair<Vector, std::vector<Vector> > SerializedTreeNode;
//typedef std::vector< SerializedTreeNode > SerializedTree;

  _stree = tree;
  drawPlannerTree=1;

}
void ForceFieldBackend::VisualizeStartGoal(const Config &p_init, const Config &p_goal)
{
  drawPlannerStartGoal = 1;
  planner_p_init = p_init;
  planner_p_goal = p_goal;
}
void ForceFieldBackend::VisualizePathSweptVolumeAtPosition(const Config &q)
{
  Robot *robot = world->robots[0];
  robot->UpdateConfig(q);
  std::vector<Matrix4> mats_config;

  for(size_t i=0;i<robot->links.size();i++) {
    Matrix4 mat = robot->links[i].T_World;
    mats_config.push_back(mat);
  }
  _mats.push_back(mats_config);
  _keyframes.push_back(q);
}

void ForceFieldBackend::VisualizePathSweptVolume(const MultiPath &path)
{
  Robot *robot = world->robots[0];

  Config qt;
  path.Evaluate(0, qt);

  double dstep = 0.01;
  double d = 0;

  while(d <= 1)
  {
    d+=dstep;
    Config qtn;
    path.Evaluate(d, qtn);
    if((qt-qtn).norm() >= sweptVolume_q_spacing)
    {
      VisualizePathSweptVolumeAtPosition(qtn);
      qt = qtn;
    }
  }
  //for(double d = 0; d <= path.Duration()+dstep; d+=dstep)
  //{
  //  while 
  //  VisualizePathSweptVolumeAtPosition(path, d);
  //}

  if(DEBUG) std::cout << "[SweptVolume] #waypoints " << _mats.size() << std::endl;
  _appearanceStack.clear();
  _appearanceStack.resize(robot->links.size());

  for(size_t i=0;i<robot->links.size();i++) {
    GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
    _appearanceStack[i]=a;
  }
  if(DEBUG) std::cout << "[SweptVolume] #geometries " << _appearanceStack.size() << std::endl;
  drawPath = 1;

}


void ForceFieldBackend::VisualizePathSweptVolume(const KinodynamicMilestonePath &path)
{
  Robot *robot = world->robots[0];

  Config qt;
  path.Eval(0, qt);

  double dstep = 0.01;
  double d = 0;

  while(d <= 1)
  {
    d+=dstep;
    Config qtn;
    path.Eval(d, qtn);
    //std::cout << d << qtn << std::endl;
    if((qt-qtn).norm() >= sweptVolume_q_spacing)
    {
      VisualizePathSweptVolumeAtPosition(qtn);
      qt = qtn;
    }
  }
  _appearanceStack.clear();
  _appearanceStack.resize(robot->links.size());

  for(size_t i=0;i<robot->links.size();i++) {
    GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
    _appearanceStack[i]=a;
  }
  drawPath = 1;
}
void ForceFieldBackend::VisualizePathSweptVolume(const std::vector<Config> &keyframes)
{
  Robot *robot = world->robots[0];

  for(int i = 0; i < keyframes.size(); i++)
  {
    VisualizePathSweptVolumeAtPosition(keyframes.at(i));
  }

  _appearanceStack.clear();
  _appearanceStack.resize(robot->links.size());

  for(size_t i=0;i<robot->links.size();i++) {
    GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
    _appearanceStack[i]=a;
  }
  drawPath = 1;
}
void ForceFieldBackend::SetIKConstraints( vector<IKGoal> constraints, string robotname){
  _constraints = constraints;
  _robotname = robotname;
  drawIKextras = 1;
}
void ForceFieldBackend::SetIKCollisions( vector<int> linksInCollision )
{
  _linksInCollision = linksInCollision;
  drawIKextras = 1;
}
bool ForceFieldBackend::OnCommand(const string& cmd,const string& args){
  stringstream ss(args);
  std::cout << "OnCommand: " << cmd << std::endl;

  if(cmd=="advance") {
    //static uint i = 0;

    //if(i>_keyframes.size()-1){
    //  return BaseT::OnCommand(cmd,args);
    //}

    //Config q = _keyframes.at(i);
    //Config q2 = _keyframes.at(i+1);
    //double dt = 1.0/_keyframes.size();
    //Config dq = (q-q2)/dt;
    ////if(i==0) dq.setZero();
    //stringstream qstr;
    //qstr<<q;

    ////string cmd( (i<=0)?("set_qv"):("append_qv") );
    //string cmd( (i<=0)?("set_q"):("append_q") );
    //sim.robotControllers[0]->SendCommand(cmd,qstr.str());
    //i++;

    std::cout << sim.time << std::endl;
    std::vector<Real> times;
    for(int i = 0; i < _keyframes.size(); i++){
      times.push_back(i);
    }
    SendLinearPath(times,_keyframes);

  }else if(cmd=="load_motion_planner") {
    std::cout << "loading file " << args.c_str() << std::endl;
  }else if(cmd=="save_motion_planner") {
    std::cout << "saving file " << args.c_str() << std::endl;
  }else{
    return BaseT::OnCommand(cmd,args);
  }
  return BaseT::OnCommand(cmd,args);
}

GLUIForceFieldGUI::GLUIForceFieldGUI(GenericBackendBase* _backend,RobotWorld* _world)
    :BaseT(_backend,_world)
{
}
void GLUIForceFieldGUI::browser_control(int control) {
  glutPostRedisplay();
  if(control==1){
    //browser
    std::string file_name = browser->get_file();
    //file = fopen(file_name.c_str(),"r"); 
    std::cout << "opening file " << file_name << std::endl;
  }
}
bool GLUIForceFieldGUI::Initialize()
{
  if(!BaseT::Initialize()) return false;
  

  panel = glui->add_rollout("Motion Planning");
  checkbox = glui->add_checkbox_to_panel(panel, "Draw Planning Tree");
  AddControl(checkbox,"draw_planner_tree");
  checkbox->set_int_val(1);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Swept Volume");
  AddControl(checkbox,"draw_path");
  checkbox->set_int_val(1);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Obstacles");
  AddControl(checkbox,"draw_rigid_objects");
  checkbox->set_int_val(1);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Start Goal Config");
  AddControl(checkbox,"draw_path_start_goal");
  checkbox->set_int_val(1);

    //AddControl(glui->add_button_to_panel(panel,"Save state"),"save_state");
  GLUI_Button* button;
  button = glui->add_button_to_panel(panel,">> Save state");
  AddControl(button, "save_motion_planner");
  button = glui->add_button_to_panel(panel,">> Load state");
  AddControl(button, "load_motion_planner");

  glui->add_button_to_panel(panel,"Quit",0,(GLUI_Update_CB)exit);
  //browser = new GLUI_FileBrowser(panel, "Loading New State", false, 1, 
      //static_cast<void(GLUIForceFieldGUI::*)(int)>(&GLUIForceFieldGUI::browser_control));
  //browser->set_h(100);
  //browser->set_w(100);
  //browser->set_allow_change_dir(0);
  //browser->fbreaddir("../data/gui");

  panel = glui->add_rollout("Fancy Decorations");
  checkbox = glui->add_checkbox_to_panel(panel, "Draw Coordinate Axes");
  AddControl(checkbox,"draw_fancy_coordinate_axes");
  checkbox->set_int_val(1);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Coordinate Axes Labels[TODO]");
  AddControl(checkbox,"draw_fancy_coordinate_axes_labels");
  checkbox->set_int_val(0);

  UpdateGUI();
  return true;

}

bool GLUIForceFieldGUI::OnCommand(const string& cmd,const string& args)
{
  return BaseT::OnCommand(cmd,args);
}

void GLUIForceFieldGUI::Handle_Keypress(unsigned char c,int x,int y)
{
    switch(c) {
      case 'h':
        printf("S: save motion planner \n");
        break;
      default:
        BaseT::Handle_Keypress(c,x,y);
    }
}




