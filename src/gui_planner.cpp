#include "gui_planner.h"
#include "drawMotionPlanner.h"

PlannerBackend::PlannerBackend(RobotWorld *world) : 
  ForceFieldBackend(world)
{
}

void PlannerBackend::AddPlannerInput(PlannerInput& _in){
  planner = new HierarchicalMotionPlanner(world, _in);
  in = _in;
}

void PlannerBackend::Start(){
  BaseT::Start();
}
bool PlannerBackend::OnCommand(const string& cmd,const string& args){
  stringstream ss(args);
  if(cmd=="hierarchy_next"){
    planner->NextPath();
  }else if(cmd=="hierarchy_previous"){
    planner->PreviousPath();
  }else if(cmd=="hierarchy_down"){
    planner->ExpandPath();
  }else if(cmd=="hierarchy_up"){
    planner->CollapsePath();
  }else return BaseT::OnCommand(cmd,args);

  SendRefresh();
  return true;

}

void PlannerBackend::RenderWorld(){

  BaseT::RenderWorld();

  DEBUG_GL_ERRORS()

  if(planner->isActive()){
    Robot* r_in = planner->GetOriginalRobot();
    const Config qi_in = planner->GetOriginalInitConfig();
    const Config qg_in = planner->GetOriginalGoalConfig();

    GLColor lightGrey(0.4,0.4,0.4,0.2);
    GLColor lightGreen(0.2,0.9,0.2,0.2);
    GLColor lightRed(0.9,0.2,0.2,0.2);
    drawRobotAtConfig(r_in, qi_in, lightGreen);
    drawRobotAtConfig(r_in, qg_in, lightRed);

    Robot* robot = planner->GetSelectedPathRobot();
    const std::vector<Config>& selected_path = planner->GetSelectedPath();
    const Config qi = planner->GetSelectedPathInitConfig();
    const Config qg = planner->GetSelectedPathGoalConfig();

    GLColor magenta(0.8,0,0.8,0.5);
    GLColor green(0.1,0.9,0.1,1);

    GLDraw::drawGLPathStartGoal(robot, qi, qg);

    GLDraw::drawPath(selected_path, green, 20);

    std::vector< std::vector<Config> > sibling_paths = planner->GetSiblingPaths();
    for(uint k = 0; k < sibling_paths.size(); k++){
      GLDraw::drawPath(sibling_paths.at(k), magenta, 10);
    }

    const SweptVolume& sv = planner->GetSelectedPathSweptVolume();
    GLDraw::drawGLPathSweptVolume(sv.GetRobot(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());
  }
  glDisable(GL_LIGHTING);

  //experiments on drawing 3d structures on fixed screen position
  double w = viewport.w;
  double h = viewport.h;

  Vector3 campos = viewport.position();
  Vector3 camdir;
  viewport.getViewVector(camdir);

  Vector3 up,down,left,right;

  viewport.getClickVector(0,h/2,left);
  viewport.getClickVector(w,h/2,right);
  viewport.getClickVector(w/2,0,down);
  viewport.getClickVector(w/2,h,up);

  up = up+campos;
  down = down+campos;
  left = left+campos;
  right = right+campos;

  glLineWidth(10);
  glPointSize(20);
  GLColor black(1,0,0);
  black.setCurrentGL();
  GLDraw::drawPoint(campos);
  GLDraw::drawPoint(up );
  GLDraw::drawPoint(down);
  GLDraw::drawPoint(right);
  GLDraw::drawPoint(left);

  GLDraw::drawLineSegment(up, left);
  GLDraw::drawLineSegment(up, right);
  GLDraw::drawLineSegment(down, right);
  GLDraw::drawLineSegment(down, left);


  GLColor grey(0.6,0.6,0.6);
  grey.setCurrentGL();
  glTranslate(left+0.5*(up-left));
  GLDraw::drawSphere(0.05,16,16);

  Vector3 tt;
  viewport.getClickVector(w/8,h/2,tt);

  glTranslate(tt + 0.1*camdir);
  GLDraw::drawSphere(0.05,16,16);
  glTranslate(tt + 0.3*camdir);
  GLDraw::drawSphere(0.05,16,16);
  glTranslate(tt + 0.4*camdir);
  GLDraw::drawSphere(0.05,16,16);

  glEnable(GL_LIGHTING);
  DEBUG_GL_ERRORS()


}
void PlannerBackend::RenderScreen(){
  BaseT::RenderScreen();
  planner->DrawGL(line_x_pos, line_y_offset);
}
bool PlannerBackend::OnIdle(){
  return BaseT::OnIdle();
}

GLUIPlannerGUI::GLUIPlannerGUI(GenericBackendBase* _backend,RobotWorld* _world, PlannerInput _in):
  BaseT(_backend,_world)
{
  in=_in;
}

bool GLUIPlannerGUI::Initialize(){
  std::cout << "Initializing GUI" << std::endl;
  if(!BaseT::Initialize()) return false;

  PlannerBackend* _backend = static_cast<PlannerBackend*>(backend);
  _backend->AddPlannerInput(in);

  AddToKeymap("left","hierarchy_previous");
  AddToKeymap("right","hierarchy_next");
  AddToKeymap("down","hierarchy_down");
  AddToKeymap("up","hierarchy_up");

  UpdateGUI();
  return true;

}
