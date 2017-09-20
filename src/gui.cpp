#include "gui.h"
#include "util.h"
#include "drawMotionPlanner.h"

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


  drawForceField = 0;
  drawWrenchField = 0;
  drawIKextras = 0;
  drawController = 1;
  drawContactDistances = 1;
  drawForceEllipsoid = 1;
  drawDistanceRobotTerrain = 1;
  drawCenterOfMassPath = 1;

  drawRigidObjects = 1;
  drawRigidObjectsEdges = 0;
  drawRigidObjectsFaces = 1;

  drawAxes = 0;
  drawAxesLabels = 0;
  drawRobot = 0;
  drawRobotExtras = 0;

  MapButtonToggle("draw_rigid_objects_faces",&drawRigidObjectsFaces);
  MapButtonToggle("draw_rigid_objects_edges",&drawRigidObjectsEdges);
  MapButtonToggle("draw_forcefield",&drawForceField);
  MapButtonToggle("draw_forceellipsoid",&drawForceEllipsoid);
  MapButtonToggle("draw_distance_robot_terrain",&drawDistanceRobotTerrain);
  MapButtonToggle("draw_com_path", &drawCenterOfMassPath);
  MapButtonToggle("draw_wrenchfield",&drawWrenchField);

  MapButtonToggle("draw_robot_extras",&drawRobotExtras);
  MapButtonToggle("draw_robot",&drawRobot);
  MapButtonToggle("draw_ik",&drawIKextras);
  MapButtonToggle("draw_fancy_coordinate_axes",&drawAxes);
  MapButtonToggle("draw_fancy_coordinate_axes_labels",&drawAxesLabels);

  drawPathShortestPath.clear();
  drawPathSweptVolume.clear();
  drawPathMilestones.clear();
  drawPathStartGoal.clear();
  drawPlannerTree.clear();
  drawPlannerSimplicialComplex.clear();

  _frames.clear();
  swept_volume_paths.clear();
}
void ForceFieldBackend::ShowSweptVolumes(){ 
  for(uint i = 0; i < drawPathSweptVolume.size(); i++){
    drawPathSweptVolume.at(i) = 1;
  }
}
void ForceFieldBackend::HideSweptVolumes(){ 
  for(uint i = 0; i < drawPathSweptVolume.size(); i++){
    drawPathSweptVolume.at(i) = 0;
  }
}
void ForceFieldBackend::ShowMilestones(){ 
  for(uint i = 0; i < drawPathMilestones.size(); i++){
    drawPathMilestones.at(i) = 1;
  }
}
void ForceFieldBackend::HideMilestones(){ 
  for(uint i = 0; i < drawPathMilestones.size(); i++){
    drawPathMilestones.at(i) = 0;
  }
}
void ForceFieldBackend::ShowPlannerTree(){ 
  for(uint i = 0; i < drawPlannerTree.size(); i++){
    drawPlannerTree.at(i) = 1;
  }
}
void ForceFieldBackend::HidePlannerTree(){ 
  for(uint i = 0; i < drawPlannerTree.size(); i++){
    drawPlannerTree.at(i) = 0;
  }
}
void ForceFieldBackend::ShowPlannerSimplicialComplex(){ 
  for(uint i = 0; i < drawPlannerSimplicialComplex.size(); i++){
    drawPlannerSimplicialComplex.at(i) = 1;
  }
}
void ForceFieldBackend::HidePlannerSimplicialComplex(){ 
  for(uint i = 0; i < drawPlannerSimplicialComplex.size(); i++){
    drawPlannerSimplicialComplex.at(i) = 0;
  }
}


///*
//############################################################################
//############################################################################
bool ForceFieldBackend::OnIdle()
{

  bool res=BaseT::OnIdle();

  if(simulate) {

    //Simulate Force Field
    // compute links COM, then get forces from field

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
  Robot *robot = world->robots[0];

  if(plannerOutput.size()>0){
    hierarchical_level = 0;

    uint N = plannerOutput.at(0).hierarchy.NumberLevels();
    for(uint k = 0; k < N; k++){
      //uint M = plannerOutput.at(0).hierarchy.NumberNodesOnLevel(k);
      hierarchical_level_nodes.push_back(0);
    }

    planner_layer_robot_idx = plannerOutput.at(0).nested_idx.back();
    planner_layer = plannerOutput.at(0).nested_idx.size()-1;
  }

  settings["dragForceMultiplier"] = 500.0;
  drawContacts = 0;
  drawWrenches = 1;
  //click_mode = ModeForceApplication;
  pose_objects = 1;

  uint Nlinks  = robot->links.size();
  std::cout << "links: " << Nlinks << std::endl;
  showLinks.resize(Nlinks); //hide certain links 

  for(uint i = 0; i < showLinks.size(); i++){
    showLinks[i] = 1;
  }
  //showLinks[15] = 1;
  //for(uint i = 43; i < 50; i++) showLinks[i] = 1;
  //for(uint i = 29; i < 36; i++) showLinks[i] = 1;

  //use custom force fields
  sim.odesim.SetGravity(Vector3(0,0,0));
  wrenchfield.init(Nlinks);

  //disable higher drawing functions
  //drawBBs,drawPoser,drawDesired,drawEstimated,drawContacts,drawWrenches,drawExpanded,drawTime,doLogging
  //drawPoser = 0;

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

  _appearanceStack.clear();
  _appearanceStack.resize(Nlinks);

  for(size_t i=0;i<Nlinks;i++) {
    GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
    if(showLinks[i]) _appearanceStack[i]=a;
  }

  drawPathShortestPath.clear();
  drawPathSweptVolume.clear();
  drawPathMilestones.clear();
  drawPathStartGoal.clear();
  drawPlannerTree.clear();
  drawPlannerSimplicialComplex.clear();

  std::cout << "Setting swept volume paths" << std::endl;
  for(uint i = 0; i < plannerOutput.size(); i++){
    drawPathShortestPath.push_back(plannerOutput.at(i).drawShortestPath);
    drawPathSweptVolume.push_back(plannerOutput.at(i).drawSweptVolume);
    drawPathStartGoal.push_back(plannerOutput.at(i).drawStartGoal);
    drawPlannerTree.push_back(plannerOutput.at(i).drawTree);
    drawPlannerSimplicialComplex.push_back(plannerOutput.at(i).drawSimplicialComplex);

    if(plannerOutput.at(i).drawMilestones) drawPathMilestones.push_back(1);
    else drawPathMilestones.push_back(0);
  }
  for(uint i = 0; i < plannerOutput.size(); i++){
    std::string dpsv = "draw_path_swept_volume_"+std::to_string(i);
    MapButtonToggle(dpsv.c_str(),&drawPathSweptVolume.at(i));
    std::string dpsp = "draw_shortest_path_"+std::to_string(i);
    MapButtonToggle(dpsp.c_str(),&drawPathShortestPath.at(i));
    std::string dpms = "draw_path_milestones_"+std::to_string(i);
    MapButtonToggle(dpms.c_str(),&drawPathMilestones.at(i));
    std::string dpsg = "draw_path_start_goal_"+std::to_string(i);
    MapButtonToggle(dpsg.c_str(),&drawPathStartGoal.at(i));
    std::string dpt = "draw_planner_tree_"+std::to_string(i);
    MapButtonToggle(dpt.c_str(),&drawPlannerTree.at(i));
    std::string dpsc = "draw_planner_simplicial_complex_"+std::to_string(i);
    MapButtonToggle(dpsc.c_str(),&drawPlannerSimplicialComplex.at(i));
  }

  std::cout << "Finished Initializing Backend" << std::endl;
}

//############################################################################
//############################################################################


void ForceFieldBackend::RenderWorld()
{
  DEBUG_GL_ERRORS()

  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND); 
  drawDesired=0;

  for(size_t i=0;i<world->terrains.size();i++){
    Terrain *terra = world->terrains[i];
    GLDraw::GeometryAppearance* a = terra->geometry.Appearance();
    a->drawFaces = false;
    a->drawEdges = false;
    a->drawVertices = false;
    if(drawRigidObjectsFaces) a->drawFaces = true;
    if(drawRigidObjectsEdges) a->drawEdges = true;
    a->vertexSize = 1;
    a->edgeSize = 10;
    terra->DrawGL();
  }

  if(drawRigidObjects){
    for(size_t i=0;i<world->rigidObjects.size();i++){
      RigidObject *obj = world->rigidObjects[i];
      GLDraw::GeometryAppearance* a = obj->geometry.Appearance();
      a->SetColor(GLColor(0.8,0.8,0.8));
      a->drawFaces = false;
      a->drawEdges = false;
      a->drawVertices = false;
      if(drawRigidObjectsFaces) a->drawFaces = true;
      if(drawRigidObjectsEdges) a->drawEdges = true;
      //a->vertexSize = 10;
      a->edgeSize = 20;
      obj->DrawGL();
    }
  }

  if(drawRobot){
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
  glEnable(GL_LIGHTING);
  glDisable(GL_BLEND); 

  BaseT::RenderWorld();

  allWidgets.Enable(&allRobotWidgets,drawPoser==1);
  allWidgets.DrawGL(viewport);
  vector<ViewRobot> viewRobots = world->robotViews;

  Robot *robot = world->robots[0];
  const ODERobot *oderobot = sim.odesim.robot(0);
  ViewRobot *viewRobot = &viewRobots[0];


  //############################################################################
  // visualize applied torque
  //############################################################################
  //SmartPointer<ContactStabilityController>& controller = *reinterpret_cast<SmartPointer<ContactStabilityController>*>(&sim.robotControllers[0]);
  //ControllerState output = controller->GetControllerState();
  //Vector torque = output.current_torque;


  //Untested/Experimental Stuff

  int drawRobotDriver = 1;

  if(drawRobotDriver){
    Vector T;
    sim.controlSimulators[0].GetActuatorTorques(T);

    Robot *robot = &oderobot->robot;
    for(uint i = 0; i < robot->drivers.size(); i++){
      RobotJointDriver driver = robot->drivers[i];
      //############################################################################
      if(driver.type == RobotJointDriver::Rotation){
      }
    //############################################################################
      if(driver.type == RobotJointDriver::Translation){
        uint didx = driver.linkIndices[0];
        uint lidx = driver.linkIndices[1];
        Frame3D Tw = robot->links[lidx].T_World;
        Vector3 pos = Tw*robot->links[lidx].com;
        Vector3 dir = Tw*robot->links[didx].w - pos;

        dir = T(i)*dir/dir.norm();

        double r = 0.05;
        glPushMatrix();
        //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,GLColor(1,0.5,0,0.7));
        glTranslate(pos);
        drawCone(-dir,2*r,8);
        glPopMatrix();

      }
    }
  }

  //############################################################################
  // Visualize
  //
  // drawrobotextras      : COM, skeleton
  // drawikextras         : contact links, contact directions
  // drawforcefield       : a flow/force field on R^3
  // drawpathsweptvolume  : swept volume along path
  // drawplannertree      : Cspace tree visualized as COM tree in W
  // drawaxes             : fancy coordinate axes
  // drawaxeslabels       : labelling of the coordinate axes [needs fixing]
  //############################################################################

  //if(drawCenterOfMassPath) GLDraw::drawCenterOfMassPathFromController(sim);
  //if(drawForceEllipsoid) GLDraw::drawForceEllipsoid(oderobot);

  if(!world->terrains.empty() && drawDistanceRobotTerrain){
    const Terrain *terrain = world->terrains[0];
    GLDraw::drawDistanceRobotTerrain(oderobot, terrain);
  }

  if(drawRobotExtras) GLDraw::drawRobotExtras(viewRobot);
  if(drawIKextras) GLDraw::drawIKextras(viewRobot, robot, _constraints, _linksInCollision, selectedLinkColor);
  if(drawForceField) GLDraw::drawForceField(wrenchfield);
  if(drawWrenchField) GLDraw::drawWrenchField(wrenchfield);

//  for(uint i = 0; i < plannerOutput.size(); i++){
//
//    Robot *robot_i = world->robots[planner_layer_robot_idx];
//
//    if(drawPathStartGoal.at(i)){
//      Config qi = plannerOutput.at(i).q_init;
//      Config qg = plannerOutput.at(i).q_goal;
//      GLDraw::drawGLPathStartGoal(robot_i, qi, qg);
//    }
//
//    SweptVolume sv = plannerOutput.at(i).GetSweptVolume(robot_i);
//
//    if(drawPathSweptVolume.at(i)){
//      GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
//    }
//
//    if(drawPathMilestones.at(i)){
//      GLDraw::drawGLPathKeyframes(robot_i, sv.GetKeyframeIndices(), sv.GetMatrices(), _appearanceStack, sv.GetColorMilestones());
//    }
//    SimplicialComplex cmplx = plannerOutput.at(i).GetSimplicialComplex();
//
//    if(drawPlannerTree.at(i)){
//      SwathVolume swv = plannerOutput.at(i).GetSwathVolume();
//      GLDraw::drawSwathVolume(swv.GetRobot(), swv.GetMatrices(), swv.GetAppearanceStack(), swv.GetColor());
//      GLDraw::drawPlannerTree(plannerOutput.at(i).GetTree());
//    }
//
//    if(drawPlannerSimplicialComplex.at(i)){
//      GLDraw::drawSimplicialComplex(cmplx);
//    }
//    if(drawPathShortestPath.at(i)){
//      GLDraw::drawShortestPath(cmplx);
//    }
//  }
  //hierarchies
  for(uint i = 0; i < plannerOutput.size(); i++){

    PathspaceHierarchy *h = &plannerOutput.at(i).hierarchy;
    Robot *robot_h = world->robots[h->GetRobotIdx(hierarchical_level)];
    //SweptVolume sv = plannerOutput.at(i).GetSweptVolume(robot_i);

    if(drawPathStartGoal.at(i)){
      Config qi = h->GetInitConfig(hierarchical_level);
      Config qg = h->GetGoalConfig(hierarchical_level);
      GLDraw::drawGLPathStartGoal(robot_h, qi, qg);
    }
    GLColor magenta(0.8,0,0.8,0.5);
    GLColor green(0.1,0.9,0.1,1);

    std::vector<int> nodes;

    if(hierarchical_level>0){
      for(uint k = 0; k < hierarchical_level-1; k++){
        nodes.push_back(hierarchical_level_nodes.at(k));
      }
      for(uint k = 0; k < h->NumberNodesOnLevel(hierarchical_level); k++){
        nodes.push_back(k);

        const std::vector<Config> &path = h->GetPathFromNodes(nodes);
        if(k == hierarchical_level_nodes.at(hierarchical_level)){
          PathNode* node = h->GetPathNodeFromNodes(nodes);
          SweptVolume sv = node->GetSweptVolume(robot_h);
          GLDraw::drawPath(path, green, 30);
          if(drawPathSweptVolume.at(i)){
            GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
          }
        }else{
          GLDraw::drawPath(path, magenta);
        }

        nodes.erase(nodes.end() - 1);
      }
    }else{
      const std::vector<Config> &path = h->GetPathFromNodes(nodes);
      PathNode* node = h->GetPathNodeFromNodes(nodes);
      SweptVolume sv = node->GetSweptVolume(robot_h);
      GLDraw::drawPath(path, green, 20);
      if(drawPathSweptVolume.at(i)){
        GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
      }
    }
  }

  if(drawAxes) drawCoordWidget(1); //void drawCoordWidget(float len,float axisWidth=0.05,float arrowLen=0.2,float arrowWidth=0.1);
  if(drawAxesLabels) GLDraw::drawAxesLabels(viewport);

  GLDraw::drawFrames(_frames, _frameLength);

  

}//RenderWorld
 
void ForceFieldBackend::RenderScreen(){
  BaseT::RenderScreen();

  int line_x_pos = 20;
  int line_y_offset = 60;
  int line_y_offset_stepsize = 20;

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

  DrawText(line_x_pos, line_y_offset, "[MotionPlanner]");
  line_y_offset += line_y_offset_stepsize;

  if(plannerOutput.size()>0){
    std::vector<int> bn = plannerOutput.at(0).cmplx.betti_numbers;
    if(bn.size()>2){
      line = "Betti      ";
      line+=(" b0: ["+std::to_string(bn[0])+"]");
      line+=(" b1: ["+std::to_string(bn[1])+"]");
      line+=(" b2: ["+std::to_string(bn[2])+"]");
      DrawText(line_x_pos, line_y_offset, line);
      line_y_offset += line_y_offset_stepsize;
    }
  }
  line = "Hierarchy    :\n";

  //line+=(" ["+std::to_string(hierarchical_level)+"]");
  uint Nlevels = plannerOutput.at(0).hierarchy.NumberLevels();

  const uint Nnodes = plannerOutput.at(0).hierarchy.NumberNodesOnLevel(1);
  //const uint selectedNode = hierarchical_level_nodes.at(hierarchical_level);

  //root node
  line = "";
  for(uint k = 0; k < 2*Nnodes; k++) line+="  ";
  line+=("<0>");
  DrawText(line_x_pos,line_y_offset,line);
  line_y_offset += line_y_offset_stepsize;

  for(uint i = 1; i < 2; i++){
    const uint selectedNode = hierarchical_level_nodes.at(i);

    line = "  ";
    for(uint k = 0; k < 2*Nnodes; k++) line+=("_");
    line+=("|");
    for(uint k = 0; k < 2*Nnodes; k++) line+=("_");
    DrawText(line_x_pos,line_y_offset,line);
    line_y_offset += line_y_offset_stepsize;

    line = " ";
    for(uint k = 0; k < Nnodes; k++) 
      line+=("    |    ");
    DrawText(line_x_pos,line_y_offset,line);
    line_y_offset += line_y_offset_stepsize;

    line = "";
    for(uint k = 0; k < Nnodes; k++){
      if(k==selectedNode && i<=hierarchical_level)
        line+=("<"+std::to_string(k)+">");
      else
        line+=("  "+std::to_string(k)+" ");
    }

    DrawText(line_x_pos,line_y_offset,line);
    line_y_offset += line_y_offset_stepsize;
  }

  // for(uint k = 0; k < ; k++){
  // }

  //for(uint k = 0; k < hierarchical_level_nodes.size(); k++){
  //  line+=(" ["+std::to_string(hierarchical_level_nodes.at(k))+"]");
  //}
  //DrawText(line_x_pos,line_y_offset,line);
  //line_y_offset += line_y_offset_stepsize;

}

void ForceFieldBackend::DrawTextVector(double xpos, double ypos, const char* prefix, Vector &v){
  stringstream ss;
  std::string line(prefix);
  ss << fixed << left << showpos << setprecision(6) << v;
  line += ss.str();
  DrawText(xpos, ypos,line);
}

void ForceFieldBackend::DrawText(int x,int y, std::string s)
{
  //GLUT_BITMAP_8_BY_13
  //GLUT_BITMAP_9_BY_15
  //GLUT_BITMAP_TIMES_ROMAN_10
  //GLUT_BITMAP_TIMES_ROMAN_24
  //GLUT_BITMAP_HELVETICA_10
  //GLUT_BITMAP_HELVETICA_12
  //GLUT_BITMAP_HELVETICA_18

  void* font = GLUT_BITMAP_HELVETICA_18;
  glPushMatrix();
  glColor3f(0,0,0);
  //glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glRasterPos2i(x,y);
  glutBitmapString(font,s.c_str());
  glEnable(GL_DEPTH_TEST);
  glPopMatrix();
}



void ForceFieldBackend::VisualizeStartGoal(const Config &p_init, const Config &p_goal)
{
  planner_p_init = p_init;
  planner_p_goal = p_goal;
}

bool ForceFieldBackend::Load(TiXmlElement *node)
{

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
      std::cout << "loaded " << keyframes.size() << " keyframes." << std::endl;
      AddPath(keyframes);
    }
    e = e->NextSiblingElement();
  }

  return true;
}
bool ForceFieldBackend::Load(const char* file)
{
  std::string pdata = util::GetDataFolder();
  std::string in = pdata+"/gui/"+file;

  std::cout << "loading data from "<<in << std::endl;

  TiXmlDocument doc(in.c_str());
  if(doc.LoadFile()){
    TiXmlElement *root = doc.RootElement();
    if(root){
      Load(root);
    }
  }else{
    std::cout << doc.ErrorDesc() << std::endl;
    std::cout << in << std::endl;
    std::cout << "ERROR" << std::endl;
  }
  //exit(0);
  return true;

}
bool ForceFieldBackend::Save(const char* file)
{
  std::string out;
  std::string pdata = util::GetDataFolder();

  if(!file){
    std::string date = util::GetCurrentTimeString();
    out = pdata+"/gui/state_"+date+".xml";
  }else{
    out = pdata+"/gui/"+file;
  }

  std::cout << "saving data to "<< out << std::endl;

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
  for(uint i = 0; i < swept_volume_paths.size(); i++){
  //
    TiXmlElement c("sweptvolume");

    std::vector<Config> keyframes = swept_volume_paths.at(i).GetKeyframes();
    for(uint i = 0; i < keyframes.size(); i++){
      TiXmlElement cc("qitem");
      stringstream ss;
      ss<< keyframes.at(i);
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    node->InsertEndChild(c);
  }

  return true;

}

//############################################################################
//############################################################################

#include <KrisLibrary/graph/Tree.h>

void ForceFieldBackend::AddPlannerOutput( PlannerOutput pout )
{
  plannerOutput.push_back(pout);

  for(uint k = 0; k < pout.removable_robot_idxs.size(); k++){
    uint idx = pout.removable_robot_idxs.at(k);
    Robot *rk = world->robots[idx];
    sim.odesim.DeleteRobot( rk->name.c_str() );
  }
}

void ForceFieldBackend::SendPlannerOutputToController()
{
  if(plannerOutput.size()>0){
    std::vector<Vector> torques = plannerOutput.at(0).GetTorques();
    for(uint i = 0; i < torques.size(); i++){
      stringstream qstr;
      qstr<<torques.at(i);
      string cmd( (i<=0)?("set_torque_control"):("append_torque_control") );
      SendCommandStringController(cmd,qstr.str());
    }
  }
}

void ForceFieldBackend::SendCommandStringController(string cmd, string arg)
{
  if(!sim.robotControllers[0]->SendCommand(cmd,arg)) {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "ERROR in controller commander" << std::endl;
    std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    throw "Controller command not supported!";
  }
}

void ForceFieldBackend::ClearPaths(){
  swept_volume_paths.clear();
}

const std::vector<Config>& ForceFieldBackend::getPathKeyFrames(uint pathid)
{
  assert(swept_volume_paths.size()>pathid);
  return swept_volume_paths.at(pathid).GetKeyframes();
}

uint ForceFieldBackend::getNumberOfPaths(){
  return this->plannerOutput.size();
}

void ForceFieldBackend::VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength)
{
  vector<Vector3> frame;
  frame.push_back(p);
  frame.push_back(e1);
  frame.push_back(e2);
  frame.push_back(e3);

  _frames.push_back(frame);
  _frameLength.push_back(frameLength);
}

//void ForceFieldBackend::AddPathInterpolate(const std::vector<Config> &keyframes, GLColor color, uint Nkeyframes_alongpath)
//{
//  std::vector<Config> interp_keyframes;
//  double minimal_distance = 0.1;
//  for(uint i = 0; i < keyframes.size()-1; i++){
//    Config q1= keyframes.at(i);
//    Config q2= keyframes.at(i+1);
//    interp_keyframes.push_back(q1);
//    double d = (q2-q1).norm();
//    std::cout << std::string(80, '-') << std::endl; 
//    std::cout << q1 << std::endl;
//    std::cout << q2 << std::endl;
//    std::cout << d << std::endl;
//
//    if(d > minimal_distance){
//      uint Npts = floor(d/minimal_distance);
//      std::cout << Npts << std::endl;
//      //std::cout << q1 << std::endl;
//      for(uint j = 0; j < Npts; j++){
//        Config qj = q1 + j*(minimal_distance/d)*(q2-q1);
//        //std::cout << qj << std::endl;
//        interp_keyframes.push_back(qj);
//      }
//      //exit(0);
//    }
//  }
//  interp_keyframes.push_back(keyframes.back());
//
//  AddPath(interp_keyframes, color, Nkeyframes_alongpath);
//}
void ForceFieldBackend::AddPath(const std::vector<Config> &keyframes, GLColor color, uint Nkeyframes_alongpath, uint robot_idx)
{
  Robot *robot = world->robots[robot_idx];
  SweptVolume sv(robot, keyframes, Nkeyframes_alongpath);
  sv.SetColor(color);
  swept_volume_paths.push_back(sv);
}

//############################################################################
//############################################################################

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
  //std::cout << "OnCommand: " << cmd << std::endl;
  if(cmd=="advance") {
    SimStep(sim.simStep);
  //}else if(cmd=="retreat") {
    //SimStep(-sim.simStep);
  }else if(cmd=="reset") {
    sim.hooks.clear();

    for(uint k = 0; k < sim.robotControllers.size(); k++){
      sim.robotControllers.at(k)->Reset();
    }

    BaseT::OnCommand(cmd,args);

    if(plannerOutput.size()>0){
      std::cout << "Reseting all robots to first planner output" << std::endl;
      for(uint k = 0; k < world->robots.size(); k++){

        Robot* robot = world->robots.at(k);
        uint ridx = plannerOutput.at(0).nested_idx.at(0);
        if(ridx == k){
          Config q = plannerOutput.at(0).nested_q_init.at(0);
          robot->UpdateConfig(q);
          robot->UpdateGeometry();
          std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
        }

        //for(uint j = 0; j < plannerOutput.at(0).nested_idx.size(); j++){
        //  uint ridx = plannerOutput.at(0).nested_idx.at(j);
        //  if(ridx == k){
        //    Config q = plannerOutput.at(0).nested_q_init.at(j);
        //    robot->UpdateConfig(q);
        //    robot->UpdateGeometry();
        //    std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
        //  }
        //}
      }
    }else{
      for(uint k = 0; k < world->robots.size(); k++){
        Robot* robot = world->robots[k];
        Config q = robot->q;
        robot->UpdateConfig(q);
        robot->UpdateGeometry();
        std::cout << "reseting robot " << robot->name << " to " << robot->q << std::endl;
      }
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

    //ODERobot *simrobot = sim.odesim.robot(0);
    //simrobot->SetConfig(planner_p_init);

  }else if(cmd=="draw_rigid_objects_faces_toggle") {
    toggle(drawRigidObjectsFaces);
  }else if(cmd=="draw_rigid_objects_edges_toggle") {
    toggle(drawRigidObjectsEdges);
  }else if(cmd=="draw_minimal"){
    drawForceField=0;
    drawWrenchField=0;
    drawForceEllipsoid=0;
    drawDistanceRobotTerrain=0;
    drawCenterOfMassPath=0;
    drawPoser = 0;
  }else if(cmd=="draw_forcefield"){
    toggle(drawForceField);
  }else if(cmd=="draw_wrenchfield"){
    toggle(drawWrenchField);
  }else if(cmd=="draw_forceellipsoid"){
    toggle(drawForceEllipsoid);
  }else if(cmd=="draw_distance_robot_terrain"){
    toggle(drawDistanceRobotTerrain);
  }else if(cmd=="draw_com_path"){
    toggle(drawCenterOfMassPath);
  }else if(cmd=="draw_planner_tree_toggle"){
    for(uint i = 0; i < drawPlannerTree.size(); i++){
      toggle(drawPlannerTree.at(i));
    }
  }else if(cmd=="draw_planner_simplicial_complex"){
    for(uint i = 0; i < drawPlannerSimplicialComplex.size(); i++){
      toggle(drawPlannerSimplicialComplex.at(i));
    }
  }else if(cmd=="draw_swept_volume"){
    for(uint i = 0; i < drawPathSweptVolume.size(); i++){
      toggle(drawPathSweptVolume.at(i));
    }
  }else if(cmd=="draw_shortest_path"){
    for(uint i = 0; i < drawPathShortestPath.size(); i++){
      toggle(drawPathShortestPath.at(i));
    }
  }else if(cmd=="load_motion_planner") {
    //glutSelectFile ("","","");//char *filename, const char *filter, const char *title)
    //std::string file_name = browser->get_file();
    //std::cout << "loading file " << file_name << std::endl;

  }else if(cmd=="save_motion_planner") {
    std::cout << "saving file " << args.c_str() << std::endl;
  }else if(cmd=="toggle_mode"){
    if(click_mode == ModeNormal){
      click_mode = ModeForceApplication;
    }else{
      click_mode = ModeNormal;
    }
    std::cout << "Changed Mode to: "<<click_mode << std::endl;
  }else if(cmd=="print_config") {
    std::cout << world->robots[0]->q <<std::endl;
  }else if(cmd=="toggle_layer") {
    int Nlayer = plannerOutput.at(0).nested_idx.size();
    if(planner_layer < Nlayer-1){
      planner_layer++;
    }else{
      planner_layer = 0;
    }
    planner_layer_robot_idx = plannerOutput.at(0).nested_idx.at(planner_layer);
    std::cout << "Switched to hierarchy level " << planner_layer << " (Robot: ";
    std::cout << planner_layer_robot_idx << " name " << world->robots[planner_layer_robot_idx]->name  << ")" << std::endl;  
  }else if(cmd=="hierarchy_next"){
    int N = plannerOutput.at(0).hierarchy.NumberNodesOnLevel(hierarchical_level);
    if(N>0){
      int node = hierarchical_level_nodes.at(hierarchical_level);
      if(node < N-1) node++;
      else node=0;
      hierarchical_level_nodes.at(hierarchical_level) = node;
    }
  }else if(cmd=="hierarchy_previous"){
    int N = plannerOutput.at(0).hierarchy.NumberNodesOnLevel(hierarchical_level);
    if(N>0){
      uint node = hierarchical_level_nodes.at(hierarchical_level);
      if(node > 0 ) node--;
      else node=N-1;
      hierarchical_level_nodes.at(hierarchical_level) = node;
    }
  }else if(cmd=="hierarchy_down"){

    std::cout << "Going deeper" << std::endl;
    std::cout << "Planning Level " << hierarchical_level << std::endl;
    std::cout << "Node           " << hierarchical_level_nodes.at(hierarchical_level) << std::endl;
    //invoke planning inception

    int N = plannerOutput.at(0).hierarchy.NumberLevels();
    if(hierarchical_level < N-1 ){
      hierarchical_level++;
    }
  }else if(cmd=="hierarchy_up"){
    int N = plannerOutput.at(0).hierarchy.NumberLevels();
    if(hierarchical_level > 0 ){
      hierarchical_level--;
    }
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
  std::cout << "Initializing GUI" << std::endl;
  if(!BaseT::Initialize()) return false;

  ForceFieldBackend* _backend = static_cast<ForceFieldBackend*>(backend);

  //std::cout << "Open Loop Controller Setup" << std::endl;
  //_backend->SendPlannerOutputToController();

  panel = glui->add_rollout("Motion Planning");
  checkbox = glui->add_checkbox_to_panel(panel, "Draw Object Edges");
  AddControl(checkbox,"draw_rigid_objects_edges");
  checkbox->set_int_val(_backend->drawRigidObjectsEdges);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Object Faces");
  AddControl(checkbox,"draw_rigid_objects_faces");
  checkbox->set_int_val(_backend->drawRigidObjectsFaces);

  uint N = _backend->getNumberOfPaths();

  std::cout << "N paths " << N << std::endl;
  for(uint i = 0; i < N; i++){
    std::string prefix = "Path "+std::to_string(i)+" :";

    std::string dpsv = "draw_path_swept_volume_"+std::to_string(i);
    std::string descr1 = prefix + "Draw Swept Volume";
    
    checkbox = glui->add_checkbox_to_panel(panel, descr1.c_str());
    AddControl(checkbox,dpsv.c_str());
    checkbox->set_int_val(_backend->drawPathSweptVolume.at(i));

    linkBox = glui->add_listbox_to_panel(panel,"Show Link",NULL);
    //toggleMeasurementDrawCheckbox = glui->add_checkbox_to_panel(panel,"Plot value");

    //Robot* robot = world->robots[_backend->plannerOutput.at(i).robot_idx];
    //uint Nlinks = robot->links.size();
    //std::cout << "path " << i << "with links " << robot->links.size() << std::endl;

    //for(size_t i=0;i<robot->links.size();i++)
    //{
    //  char buf[256];
    //  strcpy(buf,robot->linkNames[i].c_str());
    //  linkBox->add_item(i,buf);
    //}
    //AddControl(linkBox,"show_links");
    //checkbox = glui->add_checkbox_to_panel(panel, descr1.c_str());
    //AddControl(checkbox,dpsv.c_str());
    //checkbox->set_int_val(_backend->drawPathSweptVolume.at(i));

    std::string dpms = "draw_path_milestones_"+std::to_string(i);
    std::string descr2 = prefix + "Draw Milestones";
    checkbox = glui->add_checkbox_to_panel(panel, descr2.c_str());
    AddControl(checkbox,dpms.c_str());
    checkbox->set_int_val(_backend->drawPathMilestones.at(i));

    std::string dpsg = "draw_path_start_goal_"+std::to_string(i);
    std::string descr3 = prefix + "Draw Start Goal";
    checkbox = glui->add_checkbox_to_panel(panel, descr3.c_str());
    AddControl(checkbox,dpsg.c_str());
    checkbox->set_int_val(_backend->drawPathStartGoal.at(i));

    std::string dpt = "draw_planner_tree_"+std::to_string(i);
    std::string descr4 = prefix + "Draw Planner Tree";
    checkbox = glui->add_checkbox_to_panel(panel, descr4.c_str());
    AddControl(checkbox,dpt.c_str());
    checkbox->set_int_val(_backend->drawPlannerTree.at(i));

    std::string dpsc = "draw_planner_simplicial_complex_"+std::to_string(i);
    std::string descr5 = prefix + "Draw Planner SimplicialComplex";
    checkbox = glui->add_checkbox_to_panel(panel, descr5.c_str());
    AddControl(checkbox,dpsc.c_str());
    checkbox->set_int_val(_backend->drawPlannerSimplicialComplex.at(i));
  }

  //AddControl(glui->add_button_to_panel(panel,"Save state"),"save_state");
  //GLUI_Button* button;
  //button = glui->add_button_to_panel(panel,">> Save state");
  //AddControl(button, "save_motion_planner");
  //button_file_load = glui->add_button_to_panel(panel,">> Load state");
  //AddControl(button_file_load, "load_motion_planner");

  glui->add_button_to_panel(panel,"Quit",0,(GLUI_Update_CB)exit);

  //browser = new GLUI_FileBrowser(panel, "Loading New State");
  //browser->set_h(100);
  //browser->set_w(100);
  //browser->set_allow_change_dir(1);
  //browser->fbreaddir("../data/gui");

  panel = glui->add_rollout("Fancy Decorations");
  checkbox = glui->add_checkbox_to_panel(panel, "Draw Coordinate Axes");
  AddControl(checkbox,"draw_fancy_coordinate_axes");
  checkbox->set_int_val(_backend->drawAxes);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Coordinate Axes Labels[TODO]");
  AddControl(checkbox,"draw_fancy_coordinate_axes_labels");
  checkbox->set_int_val(_backend->drawAxesLabels);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Robot");
  AddControl(checkbox,"draw_robot");
  checkbox->set_int_val(_backend->drawRobot);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Force Field");
  AddControl(checkbox,"draw_forcefield");
  checkbox->set_int_val(_backend->drawForceField);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Wrench Field");
  AddControl(checkbox,"draw_wrenchfield");
  checkbox->set_int_val(_backend->drawWrenchField);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Force Ellipsoid");
  AddControl(checkbox,"draw_forceellipsoid");
  checkbox->set_int_val(_backend->drawForceEllipsoid);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Distance Robot Terrain");
  AddControl(checkbox,"draw_distance_robot_terrain");
  checkbox->set_int_val(_backend->drawDistanceRobotTerrain);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw COM Path");
  AddControl(checkbox,"draw_com_path");
  checkbox->set_int_val(_backend->drawCenterOfMassPath);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw Robot COM+Skeleton");
  AddControl(checkbox,"draw_robot_extras");
  checkbox->set_int_val(_backend->drawRobotExtras);

  checkbox = glui->add_checkbox_to_panel(panel, "Draw IK Constraints");
  AddControl(checkbox,"draw_ik");
  checkbox->set_int_val(_backend->drawIKextras);

  UpdateGUI();

//################################################################################
  //KEYMAPS
  //add keymaps to GUI
  //Handle_Keypress: show keymap on -h
  //Backend::OnCommand: handle the action
//################################################################################

  AddToKeymap("r","reset");
  //AddToKeymap("a","advance");
  //AddToKeymap("m","retreat");

  AddToKeymap("1","draw_forcefield");
  AddToKeymap("2","draw_wrenchfield");
  AddToKeymap("3","draw_forceellipsoid");
  AddToKeymap("4","draw_distance_robot_terrain");
  AddToKeymap("5","draw_com_path");

  AddToKeymap("l","toggle_layer");

  AddToKeymap("q","draw_minimal");
  AddToKeymap("T","toggle_mode");
  AddToKeymap("f","draw_rigid_objects_faces_toggle");
  AddToKeymap("e","draw_rigid_objects_edges_toggle");
  AddToKeymap("s","toggle_simulate",true);
  AddToKeymap("p","print_config",true);
  AddToKeymap("g","draw_swept_volume");
  AddToKeymap("n","draw_shortest_path");
  AddToKeymap("t","draw_planner_tree_toggle");
  AddToKeymap("G","draw_planner_simplicial_complex");
  AddToKeymap("S","make_screenshot");

  AddToKeymap("left","hierarchy_previous");
  AddToKeymap("right","hierarchy_next");
  AddToKeymap("down","hierarchy_down");
  AddToKeymap("up","hierarchy_up");

  AddButton("load_motion_planner");
  AddButton("save_motion_planner");
  AddButton("quit");
//
//              "{type:button_press,button:simulate}","toggle_simulate","",
//            "{type:button_press,button:reset}","reset","",
//            "{type:button_press,button:set_milestone}","command_pose","",
//            "{type:button_toggle,button:pose_ik_mode,checked:1}","constrain_point_mode","",
//            "{type:button_toggle,button:pose_ik,checked:0}","pose_mode","",
//            "{type:button_toggle,button:force_application_mode,checked:1}","force_application_mode","",
//            "{type:button_toggle,button:force_application_mode,checked:0}","pose_mode","",
//            "{type:button_toggle,button:do_logging,checked:1}","log_sim","simtest_log.csv",
//            "{type:button_toggle,button:do_logging,checked:0}","log_sim","",
//            "{type:button_toggle,button:do_contact_state_logging,checked:1}","log_contact_state","simtest_contact_log.csv",
//            "{type:button_toggle,button:do_contact_state_logging,checked:0}","log_contact_state","",
//            "{type:button_toggle,button:do_contact_wrench_logging,checked:1}","log_contact_wrenches","simtest_wrench_log.csv",
//            "{type:button_toggle,button:do_contact_wrench_logging,checked:0}","log_contact_wrenches","",

  return true;
}

void GLUIForceFieldGUI::AddButton(const char *key){
  AnyCollection c;
  std::string type =  "{type:button_press,button:"+std::string(key)+"}";
  //Assert(res == true);
  bool res=c.read(type.c_str());
  if(res){
    AddCommandRule(c,key,"");
  }
}

void GLUIForceFieldGUI::AddToKeymap(const char *key, const char *s, bool baseClass){
  AnyCollection c;
  std::string type =  "{type:key_down,key:"+std::string(key)+"}";
  bool res=c.read(type.c_str());
  if(res){
    AddCommandRule(c,s,"");
  }

  std::string descr(s);
  descr = std::regex_replace(descr, std::regex("_"), " ");

  _keymap[key] = descr;

  if(baseClass){
    _baseclass_keys[key] = descr;
  }

}

void GLUIForceFieldGUI::Handle_Keypress(unsigned char c,int x,int y)
{
  switch(c) {
    case 'h':
    {
      BaseT::Handle_Keypress(c,x,y);

      std::cout << "--- swept volume" << std::endl;
      for (Keymap::iterator it=_keymap.begin(); it!=_keymap.end(); ++it){
        std::cout << it->first << ": " << it->second << '\n';
      }
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
      for (Keymap::iterator it=_baseclass_keys.begin(); it!=_baseclass_keys.end(); ++it){
        if(c==*it->first){
          BaseT::Handle_Keypress(c,x,y);
          break;
        }
      }
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

