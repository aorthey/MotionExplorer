#include "util.h"
#include "gui/drawMotionPlanner.h"
#include "gui/gui.h"
#include "info.h"

#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/math3d/basis.h>
#include <KrisLibrary/math/LAPACKInterface.h>
//#include <Eigen/Core>
//#include <Eigen/SVD>

#include <tinyxml.h>
#include <iostream>
#include <fstream>


const GLColor bodyColor(0.1,0.1,0.1);
const GLColor selectedLinkColor(1.0,1.0,0.5);
const double sweptvolumeScale = 0.98;
const double sweptVolume_q_spacing = 0.01;

ForceFieldBackend::ForceFieldBackend(RobotWorld *world)
    : SimTestBackend(world)
{
  std::string guidef = util::GetDataFolder()+"/../settings/gui.xml";
  state.Load(guidef.c_str());

  for (auto it = state.variables.begin(); it != state.variables.end(); ++it) {
    GUIVariable* v = it->second;
    MapButtonToggle(v->name.c_str(), &v->active);
  }
  active_robot = 0;
}

//############################################################################
bool ForceFieldBackend::OnIdle()
{
  bool res=BaseT::OnIdle();

  if(simulate) {
    sim.odesim.SetGravity(Vector3(0,0,0));
    ODERobot *simrobot = sim.odesim.robot(active_robot);
    uint Nlinks = simrobot->robot.links.size();

    sim.hooks.clear();

    for(uint i = 0; i < Nlinks; i++){
      dBodyID bodyid = simrobot->body(i);

      if(bodyid){
        if(!simrobot->robot.IsGeometryEmpty(i)){
          Vector3 com;
          RobotLink3D *link = &simrobot->robot.links[i];
          link->GetWorldCOM(com);
          wrenchfield.setPosition(i, com);
          Vector3 linmom = simrobot->robot.GetLinearMomentum(i);
          Vector3 force = wrenchfield.getForce(com, linmom/link->mass);

          //std::cout << "link " << i << " pos " << com << " force " << force << std::endl;
          Vector3 torque(0,0,0);

          wrenchfield.setForce(i, force);
          wrenchfield.setTorque(i, torque);

          sim.hooks.push_back(new WrenchHook(bodyid, force, torque));
        }
      }
    }


    Vector3 com = simrobot->robot.GetCOM();
    //Real mass = robot->robot.GetTotalMass();
    //Matrix3 intertia = robot->robot.GetTotalInertia();

    Vector3 LM = simrobot->robot.GetLinearMomentum();
    Vector3 AM = simrobot->robot.GetAngularMomentum();

    wrenchfield.setCOMPosition(com);
    wrenchfield.setCOMLinearMomentum(LM);
    wrenchfield.setCOMAngularMomentum(AM);

    double dt=settings["updateStep"];
    Timer timer;
    SimStep(dt);
    SendRefresh();

    SensorPlotUpdate();

    Real elapsedTime = timer.ElapsedTime();
    Real remainingTime = Max(0.0,dt-elapsedTime);
    //printf("Simulated time %g took time %g, pausing for time %g\n",dt,elapsedTime,remainingTime);
    SendPauseIdle(remainingTime);

    return true;
  }
  return res;
}
//*/
//############################################################################
//############################################################################

void ForceFieldBackend::Start()
{
  BaseT::Start();

  settings["dragForceMultiplier"] = 500.0;
  drawContacts = 1;
  drawWrenches = 1;
  //click_mode = ModeForceApplication;
  click_mode = ModeNormal;
  pose_objects = 1;

  if(world->robots.size()>0){
    Robot *robot = world->robots[0];
    uint Nlinks  = robot->links.size();
    wrenchfield.init(Nlinks);
  }

  sim.odesim.SetGravity(Vector3(0,0,0));
  show_frames_per_second = true;
}

//############################################################################
//############################################################################


void ForceFieldBackend::RenderWorld()
{
  DEBUG_GL_ERRORS()
  BaseT::RenderWorld();

  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

  for(size_t i=0;i<world->terrains.size();i++){
    Terrain *terra = world->terrains[i];
    GLDraw::GeometryAppearance* a = terra->geometry.Appearance();
    a->drawFaces = false;
    a->drawEdges = false;
    a->drawVertices = false;
    if(state("draw_rigid_objects_faces")) a->drawFaces = true;
    if(state("draw_rigid_objects_edges")) a->drawEdges = true;
    a->vertexSize = 1;
    a->edgeSize = 10;
    terra->DrawGL();
  }

  for(size_t i=0;i<world->rigidObjects.size();i++){
    RigidObject *obj = world->rigidObjects[i];
    GLDraw::GeometryAppearance* a = obj->geometry.Appearance();
    a->SetColor(GLColor(0.6,0.6,0.6,1));
    a->drawFaces = false;
    a->drawEdges = false;
    a->drawVertices = false;
    if(state("draw_rigid_objects_faces")) a->drawFaces = true;
    if(state("draw_rigid_objects_edges")) a->drawEdges = true;
    a->edgeSize = 10;
    obj->DrawGL();
  }

  if(state("draw_robot")){
    for(size_t i=0;i<world->robots.size();i++) {
      if(i!=active_robot) continue;
      Robot *robot = &sim.odesim.robot(i)->robot;
      // Config q = robot->q;
      // robot->UpdateConfig(q);
      // robot->UpdateGeometry();
      // sim.odesim.robot(i)->SetConfig(q);

      //std::cout << robot->name << " selfcollisions:" << robot->SelfCollision() << std::endl;
      for(size_t j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;

        //std::cout << "robot " << i << " link " << j << " draw"  << std::endl;
        sim.odesim.robot(i)->GetLinkTransform(j,robot->links[j].T_World);
        Matrix4 mat = robot->links[j].T_World;

        glPushMatrix();
        glMultMatrix(mat);
        GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
        if(a.geom != robot->geometry[j]) a.Set(*robot->geometry[j]);
        a.SetColor(cRobot);
        a.DrawGL();
        glPopMatrix();

      }
    }
  }
  glDisable(GL_BLEND); 
  glEnable(GL_LIGHTING);

  if(state("draw_distance_robot_terrain")){
    const ODERobot *oderobot = sim.odesim.robot(0);
    for(uint k = 0; k < world->terrains.size(); k++){
      const Terrain *terrain = world->terrains[k];
      GLDraw::drawDistanceRobotTerrain(oderobot, terrain);
    }
  }

  //if(state("draw_force_ellipsoid")) GLDraw::drawForceEllipsoid(oderobot);
  if(state("draw_forcefield")) wrenchfield.DrawGL(state);
  if(state("draw_axes")) drawCoordWidget(1); //void drawCoordWidget(float len,float axisWidth=0.05,float arrowLen=0.2,float arrowWidth=0.1);
  if(state("draw_axes_labels")) GLDraw::drawAxesLabels(viewport);


}//RenderWorld
 
void ForceFieldBackend::RenderScreen(){
  
  BaseT::RenderScreen();

  if(state("draw_text_robot_info")){
    line_x_pos = 20;
    line_y_offset = 60;
    line_y_offset_stepsize = 20;

    std::string line;

    if(world->robots.size()>0){
      line = "Robots      : ";
      for(uint i = 0; i < world->robots.size(); i++){
        line += ((i>0)?" | ":"");
        line += world->robots[i]->name;
      }
      DrawText(line_x_pos,line_y_offset,line);
      line_y_offset += line_y_offset_stepsize;
    }

    if(world->terrains.size()>0){
      line = "Terrains    : ";
      for(uint i = 0; i < world->terrains.size(); i++){
        std::string geom = world->terrains[i]->geomFile;
        line += ((i>0)?" | ":"");
        line += std::string(basename(geom.c_str()));
      }
      DrawText(line_x_pos,line_y_offset,line);
      line_y_offset += line_y_offset_stepsize;
    }
    if(world->rigidObjects.size()>0){
      line = "RigidObjects: ";
      for(uint i = 0; i < world->rigidObjects.size(); i++){
        std::string geom = world->rigidObjects[i]->geomFile;
        line += ((i>0)?" | ":"");
        line += std::string(basename(geom.c_str()));
      }
      DrawText(line_x_pos,line_y_offset,line);
      line_y_offset += line_y_offset_stepsize;
    }

    line = "Mode       : ";
    line += (click_mode == ModeForceApplication?"ForceApplication":"PositionSetter");
    DrawText(line_x_pos,line_y_offset,line);
    line_y_offset += line_y_offset_stepsize;

    Vector q,dq,T;
    sim.controlSimulators[0].GetSensedConfig(q);
    sim.controlSimulators[0].GetSensedVelocity(dq);
    sim.controlSimulators[0].GetActuatorTorques(T);

    // DrawTextVector(line_x_pos, line_y_offset, "Position   :", q);
    // line_y_offset += line_y_offset_stepsize;
    // DrawTextVector(line_x_pos, line_y_offset, "Velocity   :", dq);
    // line_y_offset += line_y_offset_stepsize;
    // DrawTextVector(line_x_pos, line_y_offset, "Cmd Torque :", T);
    // line_y_offset += line_y_offset_stepsize;

  }
}

void ForceFieldBackend::DrawTextVector(double xpos, double ypos, const char* prefix, Vector &v){
  stringstream ss;
  std::string line(prefix);
  ss << fixed << left << showpos << setprecision(6) << v;
  line += ss.str();
  DrawText(xpos, ypos,line);
}

void ForceFieldBackend::DrawText(int x,int y, std::string s, void* font)
{
  //GLUT_BITMAP_8_BY_13
  //GLUT_BITMAP_9_BY_15
  //GLUT_BITMAP_TIMES_ROMAN_10
  //GLUT_BITMAP_TIMES_ROMAN_24
  //GLUT_BITMAP_HELVETICA_10
  //GLUT_BITMAP_HELVETICA_12
  //GLUT_BITMAP_HELVETICA_18

  //void* font = GLUT_BITMAP_HELVETICA_18;
  glPushMatrix();
  glColor3f(0,0,0);
  //glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glRasterPos2i(x,y);
  glutBitmapString(font,s.c_str());
  glEnable(GL_DEPTH_TEST);
  glPopMatrix();
}


bool ForceFieldBackend::Load(TiXmlElement *node)
{
  return false;
}
bool ForceFieldBackend::Load(const char* file)
{
  return false;

}
bool ForceFieldBackend::Save(const char* file)
{
  return false;

}
bool ForceFieldBackend::Save(TiXmlElement *node)
{
  return false;
}


bool ForceFieldBackend::OnCommand(const string& cmd,const string& args){
  stringstream ss(args);

  if(cmd=="advance"){
    SimStep(sim.simStep);
  }else if(cmd=="simulate"){
    state("simulate").toggle();
    simulate = state("simulate").active;
    state("draw_robot").activate();

  }else if(cmd=="reset"){
    sim.hooks.clear();

    for(uint k = 0; k < sim.robotControllers.size(); k++){
      sim.robotControllers.at(k)->Reset();
    }

    BaseT::OnCommand(cmd,args);

    for(uint k = 0; k < world->robots.size(); k++){
      Robot* robot = world->robots[k];
      Config q = robot->q;
      robot->UpdateConfig(q);
      robot->UpdateGeometry();
      std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
    }

    for(size_t i=0;i<world->robots.size();i++) {
      Robot* robot = world->robots[i];
      sim.odesim.robot(i)->SetConfig(robot->q);
      Vector dq;
      sim.odesim.robot(i)->GetVelocities(dq);
      dq.setZero();
      sim.odesim.robot(i)->SetVelocities(dq);
      robot->dq = dq;
      robotWidgets[i].SetPose(robot->q);
      robotWidgets[i].ikPoser.poseGoals.clear();
      robotWidgets[i].ikPoser.RefreshWidgets();
    }
    sim.controlSimulators[0].Init(world->robots[0], sim.odesim.robot(0), sim.robotControllers[0]);

  }else if(cmd=="draw_minimal"){
    state("draw_forcefield").deactivate();
    state("draw_wrenchfield").deactivate();
    state("draw_force_ellipsoid").deactivate();
    state("draw_distance_robot_terrain").deactivate();
    state("draw_center_of_mass_path").deactivate();
    state("draw_poser").deactivate();
  }else if(cmd=="draw_robot_next"){
    state("draw_robot").activate();
    uint N = world->robots.size();
    if(active_robot >= N-1){
      active_robot = 0;
    }else{
      active_robot++;
    }
  }else if(cmd=="draw_robot_previous"){
    state("draw_robot").activate();
    uint N = world->robots.size();
    if(active_robot > 0){
      active_robot--;
    }else{
      active_robot = N-1;
    }
  }else if(cmd=="next_mode"){
    state.NextMode();
  }else if(cmd=="previous_mode"){
    state.PreviousMode();
  }else if(cmd=="toggle_mode"){
    if(click_mode == ModeNormal){
      click_mode = ModeForceApplication;
      drawPoser = 0;
      drawDesired = 0;
      pose_objects = 0;
    }else{
      click_mode = ModeNormal;
      drawPoser = 1;
      drawDesired = 1;
      pose_objects = 1;
    }
    std::cout << "Changed Mode to: "<<click_mode << std::endl;
  }else if(cmd=="print_config") {
    if(drawDesired){
      for(uint k = 0; k < world->robots.size(); k++){
        std::cout << "Robot " << world->robots[k]->name << " : " << std::endl << robotWidgets[k].Pose() << std::endl;
      }
    }else{
      for(uint k = 0; k < world->robots.size(); k++){
        std::cout << "Robot " << world->robots[k]->name << " : " << std::endl << world->robots[k]->q <<std::endl;
      }
    }
  }else if(cmd=="print_robot_info"){
    Info info;
    info(world->robots[0]);
  }else if(state.IsToggleable(cmd.c_str())){
    state.toggle(cmd);
  }else return BaseT::OnCommand(cmd,args);

  SendRefresh();
  return true;
}
//############################################################################
// GLUIForceFieldGUI
//############################################################################

GLUIForceFieldGUI::GLUIForceFieldGUI(GenericBackendBase* _backend,RobotWorld* _world)
    :BaseT(_backend,_world)
{
}
bool GLUIForceFieldGUI::Initialize()
{
  if(!BaseT::Initialize()) return false;
  ForceFieldBackend* _backend = static_cast<ForceFieldBackend*>(backend);
  GUIState* state = &_backend->state;

  panel = glui->add_rollout("<ForceField>");
  for (auto it = state->variables.begin(); it != state->variables.end(); ++it) {
    GUIVariable* v = it->second;
    std::string descr = v->descr;
    if(v->hasKey()){
      AddToKeymap(v->key.c_str(),v->name.c_str());
      descr += " [ " + v->key + " ] ";
    }
    if(v->type == GUIVariable::Type::CHECKBOX){
      checkbox = glui->add_checkbox_to_panel(panel, descr.c_str());
      AddControl(checkbox,v->name.c_str());
      checkbox->set_int_val(v->active);
    }else if(v->type == GUIVariable::Type::BUTTON){
      //checkbox = glui->add_checkbox_to_panel(panel, descr.c_str());
      button = glui->add_button_to_panel(panel, descr.c_str());
      AddControl(button,v->name.c_str());
      AddButton(v->name.c_str());
    }else if(v->type == GUIVariable::Type::HOTKEY){
      //no graphical representation for hotkey
    }else if(v->type == GUIVariable::Type::PROPERTY){
      //no graphical representation for property
    }else if(v->type == GUIVariable::Type::SLIDER){
      spinner = glui->add_spinner_to_panel(panel, descr.c_str(), GLUI_SPINNER_FLOAT);
      spinner->set_float_val(v->value);
      spinner->set_float_limits(v->min, v->max);
      AddControl(spinner,v->name.c_str());
      //checkbox->set_int_val(v->active);
      // GLUI_Translation *GLUI::add_translation_to_panel( 
      //       GLUI_Panel *panel, const char *name, 
      //         int trans_type, float *value_ptr,
      //           int id, GLUI_CB callback 
      //             )
      // {
          //return new GLUI_Translation(panel, name, trans_type, value_ptr, id, callback);

    }else{
      std::cout << std::string(80, '#') << std::endl;
      std::cout << "variable type: " << v->type << " unknown." << std::endl;
      std::cout << "variable " << v->name << " ("<<v->descr << ")" << std::endl;
      std::cout << std::string(80, '#') << std::endl;
      exit(0);
    }
  }
  UpdateGUI();

  return true;
}

void GLUIForceFieldGUI::AddButton(const char *key){
  AnyCollection c;
  std::string type =  "{type:button_press,button:"+std::string(key)+"}";
  bool res=c.read(type.c_str());
  if(res){
    AddCommandRule(c,key,"");
  }
}

void GLUIForceFieldGUI::AddToKeymap(const char *key, const char *s){
  AnyCollection c;
  std::string type =  "{type:key_down,key:"+std::string(key)+"}";
  bool res=c.read(type.c_str());
  if(res){
    AddCommandRule(c,s,"");
  }
}

void GLUIForceFieldGUI::Handle_Keypress(unsigned char c,int x,int y)
{
  ForceFieldBackend* _backend = static_cast<ForceFieldBackend*>(backend);
  GUIState* state = &_backend->state;

  switch(c) {
    case 'h':
    {
      std::cout << std::string(80, '-') << std::endl;
      BaseT::Handle_Keypress(c,x,y);
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "<forcefield gui>" << std::endl;
      std::cout << *state << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      break;
    }
    case 'S':
    {
      SaveScreenshot();
      std::size_t pos = screenshotFile.find(".ppm");
      std::string outpng = screenshotFile.substr(0,pos)+".png";
      std::string cmd = "pnmtopng "+screenshotFile+" > "+outpng;
      int sres = system(cmd.c_str());
      IncrementStringDigits(screenshotFile);
      cmd = "mogrify -crop 650x675+6+327 "+outpng;
      int s1 = std::system(cmd.c_str());
      if(sres && s1) std::cout << "successful stored and cropped screenshot to " << outpng << std::endl;
      break;
    }
    default:
      BaseT::Handle_Keypress(c,x,y);
      break;
  }
}
bool GLUIForceFieldGUI::OnCommand(const string& cmd,const string& args)
{
  return BaseT::OnCommand(cmd,args);
}


void GLUIForceFieldGUI::Handle_Control(int id)
{
  if(controls[id]==button_file_load)
  {
    std::string file_name = browser->get_file();
    std::cout << "loading file: " << file_name << std::endl;
  }
  BaseT::Handle_Control(id);
}

