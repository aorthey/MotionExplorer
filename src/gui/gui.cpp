#include "util.h"
#include "gui/drawMotionPlanner.h"
#include "gui/gui.h"

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
  std::string guidef = util::GetDataFolder()+"/../settings/gui_default.xml";
  state.load(guidef.c_str());

  for (auto it = state.variables.begin(); it != state.variables.end(); ++it) {
    GUIVariable* v = it->second;
    MapButtonToggle(v->name.c_str(), &v->active);
  }
}

//############################################################################
bool ForceFieldBackend::OnIdle()
{
  bool res=BaseT::OnIdle();

  if(simulate) {
    sim.odesim.SetGravity(Vector3(0,0,0));
    ODERobot *robot = sim.odesim.robot(0);
    uint Nlinks = robot->robot.links.size();

    sim.hooks.clear();

    for(uint i = 0; i < Nlinks; i++){
      dBodyID bodyid = robot->body(i);

      if(bodyid){
        if(!robot->robot.IsGeometryEmpty(i)){
          Vector3 com;
          RobotLink3D *link = &robot->robot.links[i];
          link->GetWorldCOM(com);
          wrenchfield.setPosition(i, com);
          Vector3 force = wrenchfield.getForceFieldAtPosition(com);
          Vector3 torque(0,0,0);

          wrenchfield.setForce(i, force);
          wrenchfield.setTorque(i, torque);

          sim.hooks.push_back(new WrenchHook(bodyid, force, torque));
        }
      }
    }


    Vector3 com = robot->robot.GetCOM();
    //Real mass = robot->robot.GetTotalMass();
    //Matrix3 intertia = robot->robot.GetTotalInertia();

    Vector3 LM = robot->robot.GetLinearMomentum();
    Vector3 AM = robot->robot.GetAngularMomentum();

    wrenchfield.setCOMPosition(com);
    wrenchfield.setCOMLinearMomentum(LM);
    wrenchfield.setCOMAngularMomentum(AM);

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
  drawContacts = 0;
  drawWrenches = 1;
  //click_mode = ModeForceApplication;
  pose_objects = 1;

  Robot *robot = world->robots[0];
  uint Nlinks  = robot->links.size();
  wrenchfield.init(Nlinks);

  sim.odesim.SetGravity(Vector3(0,0,0));

  show_frames_per_second = true;

  //_appearanceStack.clear();
  //_appearanceStack.resize(Nlinks);
  //for(size_t i=0;i<Nlinks;i++) {
  //  GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
  //  _appearanceStack[i]=a;
  //}
}

//############################################################################
//############################################################################


void ForceFieldBackend::RenderWorld()
{
  DEBUG_GL_ERRORS()

  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

  drawDesired=0;

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
    a->SetColor(GLColor(0.6,0.6,0.6,0.2));
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
      Robot *robot = world->robots[i];
      //std::cout << robot->name << " selfcollisions:" << robot->SelfCollision() << std::endl;
      for(size_t j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;

        sim.odesim.robot(i)->GetLinkTransform(j,robot->links[j].T_World);
        Matrix4 mat = robot->links[j].T_World;

        glPushMatrix();
        glMultMatrix(mat);
        GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
        if(a.geom != robot->geometry[j]) a.Set(*robot->geometry[j]);
        a.SetColor(GLColor(0.7,0.7,0.7));
        a.DrawGL();
        glPopMatrix();

      }
    }
  }
  glDisable(GL_BLEND); 
  glEnable(GL_LIGHTING);

  if(!world->terrains.empty() && state("draw_distance_robot_terrain")){
    const ODERobot *oderobot = sim.odesim.robot(0);
    const Terrain *terrain = world->terrains[0];
    GLDraw::drawDistanceRobotTerrain(oderobot, terrain);
  }
  if(state("draw_center_of_mass_path")) GLDraw::drawCenterOfMassPathFromController(sim);
  //if(state("draw_force_ellipsoid")) GLDraw::drawForceEllipsoid(oderobot);

  if(state("draw_forcefield")) GLDraw::drawForceField(wrenchfield);
  if(state("draw_wrenchfield")) GLDraw::drawWrenchField(wrenchfield);
  if(state("draw_axes")) drawCoordWidget(1); //void drawCoordWidget(float len,float axisWidth=0.05,float arrowLen=0.2,float arrowWidth=0.1);
  if(state("draw_axes_labels")) GLDraw::drawAxesLabels(viewport);

  //GLDraw::drawFrames(_frames, _frameLength);

  //############################################################################
  // visualize applied torque
  //############################################################################
  //SmartPointer<ContactStabilityController>& controller = *reinterpret_cast<SmartPointer<ContactStabilityController>*>(&sim.robotControllers[0]);
  //ControllerState output = controller->GetControllerState();
  //Vector torque = output.current_torque;


  //Untested/Experimental Stuff

//  if(state("draw_robot_driver")){
//    Vector T;
//    sim.controlSimulators[0].GetActuatorTorques(T);
//
//    Robot *robot = &oderobot->robot;
//    for(uint i = 0; i < robot->drivers.size(); i++){
//      RobotJointDriver driver = robot->drivers[i];
//      //############################################################################
//      if(driver.type == RobotJointDriver::Rotation){
//      }
//    //############################################################################
//      if(driver.type == RobotJointDriver::Translation){
//        uint didx = driver.linkIndices[0];
//        uint lidx = driver.linkIndices[1];
//        Frame3D Tw = robot->links[lidx].T_World;
//        Vector3 pos = Tw*robot->links[lidx].com;
//        Vector3 dir = Tw*robot->links[didx].w - pos;
//
//        dir = T(i)*dir/dir.norm();
//
//        double r = 0.05;
//        glPushMatrix();
//        //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,GLColor(1,0.5,0,0.7));
//        glTranslate(pos);
//        drawCone(-dir,2*r,8);
//        glPopMatrix();
//
//      }
//    }
//  }

}//RenderWorld
 
void ForceFieldBackend::RenderScreen(){
  BaseT::RenderScreen();

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

  DrawTextVector(line_x_pos, line_y_offset, "Position   :", q);
  line_y_offset += line_y_offset_stepsize;
  DrawTextVector(line_x_pos, line_y_offset, "Velocity   :", dq);
  line_y_offset += line_y_offset_stepsize;
  DrawTextVector(line_x_pos, line_y_offset, "Cmd Torque :", T);
  line_y_offset += line_y_offset_stepsize;

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

//############################################################################
//############################################################################

#include <KrisLibrary/graph/Tree.h>

//void ForceFieldBackend::AddPlannerOutput( PlannerOutput pout )
//{
//  plannerOutput.push_back(pout);
//
//  for(uint k = 0; k < pout.removable_robot_idxs.size(); k++){
//    uint idx = pout.removable_robot_idxs.at(k);
//    Robot *rk = world->robots[idx];
//    sim.odesim.DeleteRobot( rk->name.c_str() );
//  }
//}
//
//void ForceFieldBackend::SendPlannerOutputToController()
//{
//  if(plannerOutput.size()>0){
//    std::vector<Vector> torques = plannerOutput.at(0).GetTorques();
//    for(uint i = 0; i < torques.size(); i++){
//      stringstream qstr;
//      qstr<<torques.at(i);
//      string cmd( (i<=0)?("set_torque_control"):("append_torque_control") );
//      SendCommandStringController(cmd,qstr.str());
//    }
//  }
//}
//
//void ForceFieldBackend::SendCommandStringController(string cmd, string arg)
//{
//  if(!sim.robotControllers[0]->SendCommand(cmd,arg)) {
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "ERROR in controller commander" << std::endl;
//    std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
//    std::cout << std::string(80, '-') << std::endl;
//    throw "Controller command not supported!";
//  }
//}

//void ForceFieldBackend::VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength)
//{
//  vector<Vector3> frame;
//  frame.push_back(p);
//  frame.push_back(e1);
//  frame.push_back(e2);
//  frame.push_back(e3);
//
//  _frames.push_back(frame);
//  _frameLength.push_back(frameLength);
//}
//

//############################################################################
//############################################################################


bool ForceFieldBackend::OnCommand(const string& cmd,const string& args){
  stringstream ss(args);
  if(cmd=="advance") {
    SimStep(sim.simStep);
  }else if(cmd=="reset") {
    sim.hooks.clear();

    for(uint k = 0; k < sim.robotControllers.size(); k++){
      sim.robotControllers.at(k)->Reset();
    }

    BaseT::OnCommand(cmd,args);

    //if(plannerOutput.size()>0){
    //  std::cout << "Reseting all robots to first planner output" << std::endl;
    //  for(uint k = 0; k < world->robots.size(); k++){

    //    Robot* robot = world->robots.at(k);
    //    uint ridx = plannerOutput.at(0).nested_idx.at(0);
    //    if(ridx == k){
    //      Config q = plannerOutput.at(0).nested_q_init.at(0);
    //      robot->UpdateConfig(q);
    //      robot->UpdateGeometry();
    //      std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
    //    }

    //    //for(uint j = 0; j < plannerOutput.at(0).nested_idx.size(); j++){
    //    //  uint ridx = plannerOutput.at(0).nested_idx.at(j);
    //    //  if(ridx == k){
    //    //    Config q = plannerOutput.at(0).nested_q_init.at(j);
    //    //    robot->UpdateConfig(q);
    //    //    robot->UpdateGeometry();
    //    //    std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
    //    //  }
    //    //}
    //  }
    //}else{
    //  for(uint k = 0; k < world->robots.size(); k++){
    //    Robot* robot = world->robots[k];
    //    Config q = robot->q;
    //    robot->UpdateConfig(q);
    //    robot->UpdateGeometry();
    //    std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
    //  }
    //}

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
      robotWidgets[i].SetPose(robot->q);
      robotWidgets[i].ikPoser.poseGoals.clear();
      robotWidgets[i].ikPoser.RefreshWidgets();
    }
    sim.controlSimulators[0].Init(world->robots[0], sim.odesim.robot(0), sim.robotControllers[0]);

  }else if(cmd=="draw_rigid_objects_faces") {
    state("draw_rigid_objects_faces").toggle();
  }else if(cmd=="draw_rigid_objects_edges") {
    state("draw_rigid_objects_edges").toggle();
  }else if(cmd=="draw_minimal"){
    state("draw_forcefield").deactivate();
    state("draw_wrenchfield").deactivate();
    state("draw_force_ellipsoid").deactivate();
    state("draw_distance_robot_terrain").deactivate();
    state("draw_center_of_mass_path").deactivate();
    state("draw_poser").deactivate();
  }else if(cmd=="draw_forcefield"){
    state("draw_forcefield").toggle();
  }else if(cmd=="simulate"){
    state("simulate").toggle();
    simulate = state("simulate").active;
  }else if(cmd=="draw_wrenchfield"){
    state("draw_wrenchfield").toggle();
  }else if(cmd=="draw_forceellipsoid"){
    state("draw_wrenchfield").toggle();
  }else if(cmd=="draw_distance_robot_terrain"){
    state("draw_distance_robot_terrain").toggle();
  }else if(cmd=="draw_center_of_mass_path"){
    state("draw_center_of_mass_path").toggle();
  }else if(cmd=="draw_path_space"){
    state("draw_path_space").toggle();
  }else if(cmd=="toggle_mode"){
    if(click_mode == ModeNormal){
      click_mode = ModeForceApplication;
    }else{
      click_mode = ModeNormal;
    }
    std::cout << "Changed Mode to: "<<click_mode << std::endl;
  }else if(cmd=="print_config") {
    std::cout << world->robots[0]->q <<std::endl;
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
      BaseT::Handle_Keypress(c,x,y);
      std::cout << "<forcefield gui>" << std::endl;
      std::cout << *state << std::endl;
      break;
    }
    case 'S':
    {
      SaveScreenshot();
      std::size_t pos = screenshotFile.find(".ppm");
      std::string outpng = screenshotFile.substr(0,pos)+".png";
      std::string cmd = "convert "+screenshotFile+" "+outpng;
      int sres = system(cmd.c_str());
      if(sres) std::cout << "successful stored screenshot" << std::endl;
      IncrementStringDigits(screenshotFile);
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

