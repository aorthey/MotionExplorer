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
    // std::cout << *_in.inputs.at(k) << std::endl;
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
  last_command = "";
  if(cmd=="hierarchy_next"){
    planners.at(active_planner)->Next();
    hierarchy_change = true;
    last_command = "Right";
  }else if(cmd=="hierarchy_previous"){
    planners.at(active_planner)->Previous();
    hierarchy_change = true;
    last_command = "Left";
  }else if(cmd=="hierarchy_down"){
    planners.at(active_planner)->Expand();
    hierarchy_change = true;
    last_command = "Down";
  }else if(cmd=="hierarchy_up"){
    planners.at(active_planner)->Collapse();
    hierarchy_change = true;
    last_command = "Up";
  }else if(cmd=="benchmark"){
  }else if(cmd=="planner_step"){
    last_command = "Step";
    planners.at(active_planner)->Step();
  }else if(cmd=="planner_clear"){
    last_command = "Clear";
    planners.at(active_planner)->Clear();
    hierarchy_change = true;
  }else if(cmd=="planner_advance_until_solution"){
    last_command = "Plan";
    planners.at(active_planner)->AdvanceUntilSolution();
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
    if(path){
      path->Smooth(true);
    }
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
    state("draw_planner_text").toggle();
  }else if(cmd=="draw_path_modus"){
    draw_path_modus++;
    if(draw_path_modus > 2) draw_path_modus = 0;
    switch (draw_path_modus) {
      case 0:
        state("draw_path").activate();
        state("draw_path_partial").deactivate();
        state("draw_explorer_unselected_paths").activate();
        break;
      case 1:
        state("draw_path").activate();
        state("draw_path_partial").activate();
        state("draw_explorer_unselected_paths").activate();
        break;
      default:
        state("draw_path").deactivate();
        state("draw_path_partial").deactivate();
        state("draw_explorer_unselected_paths").deactivate();
        break;
    }
  }else if(cmd=="draw_play_path"){
    last_command = "Execute";
    state("draw_play_path").toggle();
    simulate = 0;
    if(state("draw_play_path")){
      SendPauseIdle(0);
    }else{
      SendPauseIdle();
    }
  }else if(cmd=="simulate_controller"){
    last_command = "Execute";
    //get controls
    MotionPlanner* planner = planners.at(active_planner);
    path = planner->GetPath();
    if(path){
      controller_ = sim.robotControllers[0];

      path->SendToController(controller_);

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
      // if(state("simulate") != state("draw_play_path")){
      //   state("simulate").toggle();
      // }

      state("draw_play_path").activate();
      state("simulate").activate();
      state("draw_robot").activate();

      if(state("draw_play_path")){
        SendPauseIdle(0);
      }else{
        SendPauseIdle();
      }
      // t=0;
    }else{
      std::cout << "no active control path" << std::endl;
    }

  }else if(cmd=="save_current_path"){
    //state("save_current_path").activate();
    path = planners.at(active_planner)->GetPath();
    if(path)
    {
      std::string fname = "../data/paths/"+getRobotEnvironmentString()+".path";
      path->Save(fname.c_str());
      std::cout << "save current path (" << path->GetNumberOfMilestones() 
        << " states) to : " << fname << std::endl;
    }else{
      std::cout << "cannot save non-existing path." << std::endl;
    }
  }else if(cmd=="load_current_path"){
    MotionPlanner* planner = planners.at(active_planner); // std::string fn = planner->GetInput().name_loadPath;
    std::string fname = "../data/paths/"+getRobotEnvironmentString()+".path";
    if(!path)
    {
        CSpaceOMPL* cspace = planner->GetCSpace();
        path = new PathPiecewiseLinear(cspace);
        if(planner->GetInput().name_loadPath != "")
        {
            fname = planner->GetInput().name_loadPath;
        }
    }
    path->Load(fname.c_str());
    std::cout << "load current path from : " << fname << std::endl;
  }else if(cmd=="save_view"){
    last_command = "SaveView";
    std::string fname = "../data/viewport/"+getRobotEnvironmentString()+".viewport";
    std::cout << "Save: " << fname << std::endl;
    BaseT::OnCommand("save_view",fname.c_str());
  }else if(cmd=="load_view" || cmd=="reset_view"){
    last_command = "LoadView";
    std::string fname = "../data/viewport/"+getRobotEnvironmentString()+".viewport";
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

std::string PlannerBackend::getRobotEnvironmentString()
{
    std::string rname, tname;

    if(world->robots.size()>0){
        rname = world->robots[0]->name;
    }
    if(world->terrains.size()>0){
        tname = world->terrains[0]->name;
    }else{
      if(world->rigidObjects.size()>0){
          tname = world->rigidObjects[0]->name;
      }
    }
    return rname+"_"+tname;
}

void PlannerBackend::CenterCameraOn(const Vector3& v)
{
  Math3D::AABB3D box(v,v);
  GLNavigationBackend::CenterCameraOn(box);
}

bool PlannerBackend::OnIdle()
{
  bool res = BaseT::OnIdle();
  if(planners.empty()) return res;

  MotionPlanner* planner = planners.at(active_planner);

  if(!planner->isActive()) return false;

  SendRefresh();
  if(planner->hasChanged())
  {
      SendRefresh();
  }

  if(state("draw_play_path"))
  {
    if(t<=0){
      path = planner->GetPath();
    }
    if(path)
    {
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
    }

    static PathPiecewiseLinear *path;
    if(state("draw_play_path")){
      if(t<=0){
        path = planner->GetPath();
        // if(!path){
          // std::cout << "No path available." << std::endl;
        // }
      }
      if(t>0 && path!=nullptr){
        path->DrawGL(state, t);
      }
    }
  }
  //if(planner->GetInput().kinodynamic){
  //  if(state("draw_controller_com_path")) GLDraw::drawCenterOfMassPathFromController(sim);

  //  SmartPointer<ContactStabilityController>& controller = *reinterpret_cast<SmartPointer<ContactStabilityController>*>(&sim.robotControllers[0]);
  //  ControllerState output = controller->GetControllerState();
  //  Vector torque = output.current_torque;

  //  //Untested/Experimental Stuff
  //  if(state("draw_controller_driver")){
  //    Vector T;
  //    sim.controlSimulators[0].GetActuatorTorques(T);
  //    Robot *robot = &sim.odesim.robot(0)->robot;

  //    Vector3 dir;
  //    for(uint k = 0; k < 3; k++){
  //      dir[k] = T[k];
  //    }
  //    //dir = T(i)*dir/dir.norm();

  //    const RobotJointDriver& driver = robot->drivers[0];
  //    //uint didx = driver.linkIndices[0];
  //    uint lidx = driver.linkIndices[1];
  //    Frame3D Tw = robot->links[lidx].T_World;
  //    Vector3 pos = Tw*robot->links[lidx].com;

  //    double r = 0.05;
  //    glPushMatrix();
  //    glTranslate(pos);
  //    drawCone(-dir,2*r,8);
  //    glPopMatrix();
  //  }
  //}
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
void DrawGLRoundedCorner(int x, int y, double sa, double arc, float r) {
    // centre of the arc, for clockwise sense
    float cent_x = x;// + r * cos(sa + 0.5*Pi);
    float cent_y = y;// + r * sin(sa + 0.5*Pi);
    // glVertex2f(cent_x, cent_y);

    // build up piecemeal including end of the arc
    int n = ceil(16 * (arc / (2*Pi)));
    for (int i = 0; i <= n; i++) {
        double ang = sa + (arc-sa) * (double)i  / (double)n;
        float next_x = cent_x + r * cos(ang);
        float next_y = cent_y - r * sin(ang);
        glVertex2f(next_x, next_y);
    }
}
void PlannerBackend::RenderCommand(const std::string &cmd)
{
    double radius = 60;
    double y = viewport.h - 100;
    double x = line_x_pos + 1.5*radius;

    // int numIncrements = 16;
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    GLDraw::GLColor boundColor, textColor, nodeColor;
    textColor.set(0.1,0.1,0.1);
    nodeColor.set(0.8,0.8,0.8,0.5);
    boundColor.set(0.2,0.2,0.2,0.5);
    
    //############################################################################
		nodeColor.setCurrentGL();
    double dx = 0.2*radius;
    double dr = radius-dx;
    glBegin(GL_POLYGON);
    DrawGLRoundedCorner(x-dr, y-dr, Pi/2  , Pi    , dx);
    DrawGLRoundedCorner(x-dr, y+dr, Pi, 3*Pi/2  , dx);
    DrawGLRoundedCorner(x+dr, y+dr, 3*Pi/2 , 2*Pi, dx);
    DrawGLRoundedCorner(x+dr, y-dr, 0     , Pi/2  , dx);
    glEnd();

		boundColor.setCurrentGL();
    glBegin(GL_LINE_LOOP);
    DrawGLRoundedCorner(x-dr, y-dr, Pi/2  , Pi    , dx);
    DrawGLRoundedCorner(x-dr, y+dr, Pi, 3*Pi/2  , dx);
    DrawGLRoundedCorner(x+dr, y+dr, 3*Pi/2 , 2*Pi, dx);
    DrawGLRoundedCorner(x+dr, y-dr, 0     , Pi/2  , dx);
    glEnd();

    textColor.setCurrentGL();
    char buf[64];
    void* font=GLUT_BITMAP_HELVETICA_18;
    sprintf(buf,"%s\n", cmd.c_str());

    unsigned Ls = cmd.size();
    double offset = 0.5*Ls*10;

    glRasterPos2d(x-offset,y); 
    glutBitmapString(font,buf);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

}

void PlannerBackend::RenderScreen(){
  BaseT::RenderScreen();
  if(state("draw_planner_text")){
    std::string line;
    line = "Planners       : ";
    DrawText(line_x_pos,line_y_offset,line);
    line_y_offset += line_y_offset_stepsize;

    for(uint k = 0; k < planners.size(); k++){
      uint number_of_stratifications = planners.at(k)->GetInput().stratifications.size();
      line = "               ";
      if(k==active_planner) line += "[";

      line += planners.at(k)->getName() + " ";
      if(number_of_stratifications<=1){
        uint plvl = planners.at(k)->GetInput().stratifications.front().layers.size();
        line += "(" + std::to_string(plvl) + " levels)";
      }else{
        line += "(" + std::to_string(number_of_stratifications) + " algorithms)";
      }
      if(k==active_planner) line += "]";

      DrawText(line_x_pos,line_y_offset,line);
      line_y_offset += line_y_offset_stepsize;
    }

  }

  if(planners.size()>0){
    //display time
    double time = planners.at(active_planner)->getLastIterationTime();
    std::stringstream timeStream;
    timeStream << std::fixed << std::setprecision(2) << time;
    std::string line = "Planner Time: ";
    line += timeStream.str()+"s";
    DrawText(line_x_pos, line_y_offset, line);
    line_y_offset += line_y_offset_stepsize;

    if(state("draw_planner_minima_tree")){
        planners.at(active_planner)->DrawGLScreen(line_x_pos, line_y_offset);
        line_y_offset += line_y_offset_stepsize;
    }
  }
  if(state("draw_planner_last_command") && last_command!=""){
      RenderCommand(last_command);
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
