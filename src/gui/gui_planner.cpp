#include "gui/gui_planner.h"
#include "elements/path_pwl.h"
#include "gui/drawMotionPlanner.h"

PlannerBackend::PlannerBackend(RobotWorld *world) : 
  ForceFieldBackend(world)
{
  active_planner=0;
  t = 0;
}

void PlannerBackend::AddPlannerInput(PlannerMultiInput& _in){
  for(uint k = 0; k < _in.inputs.size(); k++){
    //std::cout << *_in.inputs.at(k) << std::endl;
    planners.push_back( new MotionPlanner(world, *_in.inputs.at(k)) );
  }
}

void PlannerBackend::Start(){
  BaseT::Start();
  drawPoser = 0;
  drawDesired = 0;
  pose_objects = 0;
  drawBBs = 0;
  show_view_target = 0;

  if(state("load_view_from_file")){
    this->OnCommand("load_view","");
  }


}

bool PlannerBackend::OnCommand(const string& cmd,const string& args){

  if(planners.empty()) return BaseT::OnCommand(cmd, args);

  stringstream ss(args);
  bool hierarchy_change = false;
  if(cmd=="hierarchy_next"){
    planners.at(active_planner)->Next();
    hierarchy_change = true;
  }else if(cmd=="hierarchy_previous"){
    planners.at(active_planner)->Previous();
    hierarchy_change = true;
  }else if(cmd=="hierarchy_down"){
    planners.at(active_planner)->Expand();
    hierarchy_change = true;
  }else if(cmd=="hierarchy_up"){
    planners.at(active_planner)->Collapse();
    hierarchy_change = true;
  }else if(cmd=="benchmark"){
  }else if(cmd=="planner_step"){
    planners.at(active_planner)->Step();
    planners.at(active_planner)->ExpandFull();
  }else if(cmd=="planner_step_one_level"){
    planners.at(active_planner)->StepOneLevel();
    planners.at(active_planner)->Expand();
    hierarchy_change = true;
  }else if(cmd=="planner_clear"){
    std::cout << "CLEAR PLanner" << std::endl;
    planners.at(active_planner)->Clear();
    hierarchy_change = true;
  }else if(cmd=="planner_advance_until_solution"){
    planners.at(active_planner)->AdvanceUntilSolution();
    planners.at(active_planner)->ExpandFull();
    hierarchy_change = true;
  }else if(cmd=="next_planner"){
    if(active_planner<planners.size()-1) active_planner++;
    else active_planner = 0;
    hierarchy_change = true;
    path = planners.at(active_planner)->GetPath();
  }else if(cmd=="previous_planner"){
    if(active_planner>0) active_planner--;
    else active_planner = planners.size()-1;
    hierarchy_change = true;
    path = planners.at(active_planner)->GetPath();
  }else if(cmd=="smooth_path"){
    path = planners.at(active_planner)->GetPath();
    if(path) path->Smooth();
  }else if(cmd=="draw_cover_single_open_set"){
    draw_cover_all_open_sets = false;
    draw_cover_active_open_set++;
  }else if(cmd=="planner_draw_start_goal_configuration"){
    state("planner_draw_goal_configuration").toggle();
    state("planner_draw_start_configuration").toggle();
  }else if(cmd=="planner_draw_goal_configuration"){
    state("planner_draw_goal_configuration").toggle();
  }else if(cmd=="planner_draw_start_configuration"){
    state("planner_draw_start_configuration").toggle();
  }else if(cmd=="path_speed_increase"){
    MotionPlanner* planner = planners.at(active_planner);
    GUIVariable &v = state("path_speed");
    v.value = planner->GetInput().pathSpeed;
    v.value = min(v.max, v.value + v.step);
    planner->GetInput().pathSpeed = v.value;
    std::cout << "Increased path speed to " << v.value << std::endl;
  }else if(cmd=="path_speed_decrease"){
    MotionPlanner* planner = planners.at(active_planner);
    GUIVariable &v = state("path_speed");
    v.value = planner->GetInput().pathSpeed;
    v.value = max(v.min, v.value - v.step);
    planner->GetInput().pathSpeed = v.value;
    std::cout << "Decreased path speed to " << v.value << std::endl;
  }else if(cmd=="draw_text"){
    state("draw_text_robot_info").toggle();
    state("draw_text_planner").toggle();
  }else if(cmd=="draw_play_path"){
    state("draw_play_path").toggle();
    simulate = 0;
    if(state("draw_play_path")){
      SendPauseIdle(0);
    }else{
      SendPauseIdle();
    }
  }else if(cmd=="simulate_controller"){
    //get controls
    MotionPlanner* planner = planners.at(active_planner);
    path = planner->GetPath();
    if(path){
      SmartPointer<RobotController> ctrl = sim.robotControllers[0];

      path->SendToController(ctrl);

      Config q = path->Eval(0);
      Config dq = path->EvalVelocity(0);

      //Robot* robot = &oderobot->robot;
      ODERobot *oderobot = sim.odesim.robot(0);
      oderobot->robot.q = q;
      oderobot->robot.dq = dq;
      oderobot->robot.UpdateDynamics();

      oderobot->SetConfig(q);
      oderobot->SetVelocities(dq);

      Robot* robot = world->robots[0];
      robot->q = q;
      robot->dq = dq;
      robot->UpdateDynamics();

      //activate simulation
      state("simulate").toggle();
      simulate = state("simulate").active;
      if(simulate) state("draw_robot").activate();
    }else{
      std::cout << "no active control path" << std::endl;
    }

  }else if(cmd=="save_current_path"){
    //state("save_current_path").activate();
    if(path)
    {
      std::string fn = "mypath.xml";
      path->Save(fn.c_str());
      std::cout << "save current path to : " << fn << std::endl;
    }else{
      std::cout << "cannot save non-existing path." << std::endl;
    }
  }else if(cmd=="load_current_path"){
    std::cout << "load_current_path: NYI" << std::endl;
    // if(!path)
    // {
    //   MotionPlanner* planner = planners.at(active_planner);
    //   path = new PathPiecewiseLinear();
    // }
    // {
    //   std::string fn = "mypath.xml";
    //   path->Load(fn.c_str());
    //   std::cout << "load current path from : " << fn << std::endl;
    // }
  }else if(cmd=="save_view"){
    Robot *robot = world->robots[active_robot];
    std::string rname = robot->name;
    std::string fname = "../data/viewport/robot_"+rname+".viewport";
    BaseT::OnCommand("save_view",fname.c_str());
  }else if(cmd=="load_view" || cmd=="reset_view"){
    Robot *robot = world->robots[active_robot];
    std::string rname = robot->name;
    std::string fname = "../data/viewport/robot_"+rname+".viewport";
    std::cout << "Load: " << fname << std::endl;
    BaseT::OnCommand("load_view",fname.c_str());
  }else return BaseT::OnCommand(cmd,args);

  if(hierarchy_change){
    t=0;
    if(state("draw_play_path")){
      SendPauseIdle();
      state("draw_play_path").deactivate();
    }
  }

  SendRefresh();
  return true;
}
void PlannerBackend::CenterCameraOn(const Vector3& v){
  //Vector3 bmin = v-epsilon;
  //Vector3 bmax = v-epsilon;
  Math3D::AABB3D box(v,v);
  GLNavigationBackend::CenterCameraOn(box);
}

bool PlannerBackend::OnIdle(){
  bool res = BaseT::OnIdle();
  if(planners.empty()) return res;

  MotionPlanner* planner = planners.at(active_planner);
  if(state("draw_play_path")){
    if(t<=0){
      path = planner->GetPath();
    }
    if(path){
      double T = path->GetLength();
      double tstep = planner->GetInput().pathSpeed*T/1000;
      //std::cout << "play path: " << t << "/" << T << std::endl;
      if(t>=T){
        t=0;
        SendPauseIdle();
      }else{
        t+=tstep;
        SendRefresh();
      }
      if(state("draw_path_autofocus")){
        Vector3 v = path->EvalVec3(t);
        CenterCameraOn(v);
      }
    }
    return true;
  }
  if(state("simulate_controller")){
  }
  return res;

}

void PlannerBackend::RenderWorld(){

  BaseT::RenderWorld();

  if(planners.empty()) return;

  MotionPlanner* planner = planners.at(active_planner);

  if(planner->isActive()){

    planner->DrawGL(state);

    if(state("draw_planner_surface_normals")){
      glDisable(GL_LIGHTING);
      glEnable(GL_BLEND); 
      for(uint k = 0; k < world->terrains.size(); k++)
      {
        SmartPointer<Terrain> terrain = world->terrains.at(k);
        ManagedGeometry& geometry = terrain->geometry;
        const Meshing::TriMesh& trimesh = geometry->AsTriangleMesh();
        uint N = trimesh.tris.size();
        //Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(N,N);
        glLineWidth(3);
        //GLColor cNormal(0.8,0.5,0.5,0.5);
        setColor(magenta);
        for(uint i = 0; i < N; i++){
          Vector3 vA = trimesh.TriangleVertex(i,0);
          Vector3 vB = trimesh.TriangleVertex(i,1);
          Vector3 vC = trimesh.TriangleVertex(i,2);
          Vector3 ni = trimesh.TriangleNormal(i);
          //compute incenter
          // double c = vA.distance(vB);
          // double b = vA.distance(vC);
          // double a = vB.distance(vC);
          //Vector3 incenter = (a*vA+b*vB+c*vC)/(a+b+c);
          Vector3 centroid = (vA+vB+vC)/3.0;
          //drawLineSegment(incenter, incenter - ni);
          drawLineSegment(centroid, centroid + ni);

          //how to obtain correct orientation!? (locally orientable? it should
          //be straightforward to determine if a normal component is enclosed in
          //the object)
        }
      }
      glEnable(GL_LIGHTING);
      glDisable(GL_BLEND); 
    }

    if(state("draw_planner_bounding_box")){
      Config min = planner->GetInput().se3min;
      Config max = planner->GetInput().se3max;

      //minimal width of bounding box, otherwise near zero volumes are not
      //visible
      double minimal_width = 0.1;
      for(uint k = 0; k < 3; k++){
        double d = fabs(max(k)-min(k));
        if(d<minimal_width){
          max(k)+=minimal_width/2.0;
          min(k)-=minimal_width/2.0;
        }
      }

      glDisable(GL_LIGHTING);
      glEnable(GL_BLEND); 
      GLColor lightgrey(0.5,0.5,0.5,0.8);
      lightgrey.setCurrentGL();
      GLDraw::drawBoundingBox(Vector3(min(0),min(1),min(2)), Vector3(max(0),max(1),max(2)));
      glEnable(GL_LIGHTING);
      glDisable(GL_BLEND); 

      // uint N = 8;
      // world->lights.resize(N);
      // for(uint k = 0; k < N; k++){
      //   world->lights[k].setColor(GLColor(1,1,1));
      // }
      // world->lights[0].setPointLight(Vector3(min(0),min(1),min(2)));
      // world->lights[1].setPointLight(Vector3(min(0),min(1),max(2)));
      // world->lights[2].setPointLight(Vector3(min(0),max(1),min(2)));
      // world->lights[3].setPointLight(Vector3(min(0),max(1),max(2)));
      // world->lights[4].setPointLight(Vector3(max(0),min(1),min(2)));
      // world->lights[5].setPointLight(Vector3(max(0),min(1),max(2)));
      // world->lights[6].setPointLight(Vector3(max(0),max(1),min(2)));
      // world->lights[7].setPointLight(Vector3(max(0),max(1),max(2)));

    }
    static PathPiecewiseLinear *path;
    if(state("draw_play_path")){
      if(t<=0){
        path = planner->GetPath();
        if(!path){
          std::cout << "No path available." << std::endl;
        }else{
          //std::cout << *path << std::endl;
        }
      }
    }
    if(t>0 && path){
      path->DrawGL(state, t);
    }
  }
  if(planner->GetInput().kinodynamic){
    if(state("draw_controller_com_path")) GLDraw::drawCenterOfMassPathFromController(sim);

    SmartPointer<ContactStabilityController>& controller = *reinterpret_cast<SmartPointer<ContactStabilityController>*>(&sim.robotControllers[0]);
    ControllerState output = controller->GetControllerState();
    Vector torque = output.current_torque;

    //Untested/Experimental Stuff
    if(state("draw_controller_driver")){
      Vector T;
      sim.controlSimulators[0].GetActuatorTorques(T);
      Robot *robot = &sim.odesim.robot(0)->robot;

      Vector3 dir;
      for(uint k = 0; k < 3; k++){
        dir[k] = T[k];
      }
      //dir = T(i)*dir/dir.norm();

      const RobotJointDriver& driver = robot->drivers[0];
      //uint didx = driver.linkIndices[0];
      uint lidx = driver.linkIndices[1];
      Frame3D Tw = robot->links[lidx].T_World;
      Vector3 pos = Tw*robot->links[lidx].com;

      double r = 0.05;
      glPushMatrix();
      glTranslate(pos);
      drawCone(-dir,2*r,8);
      glPopMatrix();
    }
  }
}

 // //experiments on drawing 3d structures on fixed screen position
 // double w = viewport.w;
 // double h = viewport.h;

 // Vector3 campos = viewport.position();
 // Vector3 camdir;
 // viewport.getViewVector(camdir);

 // Vector3 up,down,left,right;

 // viewport.getClickVector(0,h/2,left);
 // viewport.getClickVector(w,h/2,right);
 // viewport.getClickVector(w/2,0,down);
 // viewport.getClickVector(w/2,h,up);

 // up = up+campos;
 // down = down+campos;
 // left = left+campos;
 // right = right+campos;

 // glLineWidth(10);
 // glPointSize(20);
 // GLColor black(1,0,0);
 // black.setCurrentGL();
 // GLDraw::drawPoint(campos);
 // GLDraw::drawPoint(up );
 // GLDraw::drawPoint(down);
 // GLDraw::drawPoint(right);
 // GLDraw::drawPoint(left);

 // GLDraw::drawLineSegment(up, left);
 // GLDraw::drawLineSegment(up, right);
 // GLDraw::drawLineSegment(down, right);
 // GLDraw::drawLineSegment(down, left);


 // GLColor grey(0.6,0.6,0.6);
 // grey.setCurrentGL();
 // glTranslate(left+0.5*(up-left));
 // GLDraw::drawSphere(0.05,16,16);

 // Vector3 tt;
 // viewport.getClickVector(w/8,h/2,tt);

 // glTranslate(tt + 0.1*camdir);
 // GLDraw::drawSphere(0.05,16,16);
 // glTranslate(tt + 0.3*camdir);
 // GLDraw::drawSphere(0.05,16,16);
 // glTranslate(tt + 0.4*camdir);
 // GLDraw::drawSphere(0.05,16,16);

 // glEnable(GL_LIGHTING);
void PlannerBackend::RenderScreen(){
  BaseT::RenderScreen();
  if(state("draw_text_planner")){
    std::string line;
    line = "Planners       : ";
    DrawText(line_x_pos,line_y_offset,line);
    line_y_offset += line_y_offset_stepsize;

    for(uint k = 0; k < planners.size(); k++){
      line = "               ";
      if(k==active_planner) line += "[";
      line += planners.at(k)->getName() + " ";
      if(k==active_planner) line += "]";

      DrawText(line_x_pos,line_y_offset,line);
      line_y_offset += line_y_offset_stepsize;
    }

    if(planners.size()>0){
      planners.at(active_planner)->DrawGLScreen(line_x_pos, line_y_offset);
    }
  }
}

GLUIPlannerGUI::GLUIPlannerGUI(GenericBackendBase* _backend,RobotWorld* _world):
  BaseT(_backend,_world)
{
}

void GLUIPlannerGUI::AddPlannerInput(PlannerMultiInput& _in){
  PlannerBackend* _backend = static_cast<PlannerBackend*>(backend);
  _backend->AddPlannerInput(_in);
}

bool GLUIPlannerGUI::Initialize(){
  if(!BaseT::Initialize()) return false;

  //PlannerBackend* _backend = static_cast<PlannerBackend*>(backend);

  UpdateGUI();
// save/load planner:
  //AddControl(glui->add_button_to_panel(panel,"Save state"),"save_state");
  //GLUI_Button* button;
  //button = glui->add_button_to_panel(panel,">> Save state");
  //AddControl(button, "save_motion_planner");
  //button_file_load = glui->add_button_to_panel(panel,">> Load state");
  //AddControl(button_file_load, "load_motion_planner");
// open file from filesystem
  //browser = new GLUI_FileBrowser(panel, "Loading New State");
  //browser->set_h(100);
  //browser->set_w(100);
  //browser->set_allow_change_dir(1);
  //browser->fbreaddir("../data/gui");

  return true;

}
