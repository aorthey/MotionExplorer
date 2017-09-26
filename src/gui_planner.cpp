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
  //const SweptVolume& sv = planner->GetSelectedPathSweptVolume();
  //GLDraw::drawGLPathKeyframes(sv.GetRobot(), sv.GetKeyframeIndices(), sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColorMilestones());

    //Camera::Viewport viewport;
      //Camera::CameraController_Orbit camera;

  //experiments on drawing 3d structures on fixed screen position
  double scale = viewport.scale;
  double w = viewport.w;
  double h = viewport.h;
  double x = viewport.x;
  double y = viewport.y;
  double n = viewport.n;
  double f = viewport.f;
  Vector3 campos = viewport.position();
  Vector3 camdir;
  viewport.getViewVector(camdir);
  //camdir = camdir / camdir.norm();

  Vector3 xdir;
  xdir.set(viewport.xDir());
  xdir.inplaceNegative();
  //xdir /= xdir.norm();
  Vector3 zdir;
  zdir.set(viewport.zDir());
  zdir.inplaceNegative();
  Vector3 ydir;
  ydir.set(viewport.yDir());
  ydir.inplaceNegative();
  //ydir /= ydir.norm();

  xdir *= scale;
  ydir *= scale;
  zdir *= scale;




  //Vector3 left= campos + xdir + zdir;
  //Vector3 right= campos - xdir + camdir;

  //Vector3 up= campos - ydir + camdir;
  //Vector3 down= campos + ydir + camdir;

  Vector3 up,down,left,right,updir;
  //viewport.getClickSource(w/4,h/4,up);
  viewport.getClickVector(w,0,up);

  std::cout << up << "<>" << campos << std::endl;
  //up = up+updir;
  //down = down+campos;
  //right = right+campos;

  glLineWidth(10);
  glPointSize(20);
  GLColor black(1,0,0);
  black.setCurrentGL();
  //GLDraw::drawPoint(campos + camdir);
  GLDraw::drawPoint(campos + camdir);
  GLDraw::drawPoint(campos + up + camdir);
  //GLDraw::drawPoint(down);
  //GLDraw::drawPoint(right);
  GLDraw::drawLineSegment(up+camdir, campos+camdir);
  //GLDraw::drawLineSegment(up, right);
  //GLDraw::drawLineSegment(down, right);

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
