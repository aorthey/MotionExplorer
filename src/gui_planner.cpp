#include "gui_planner.h"
#include "drawMotionPlanner.h"

PlannerBackend::PlannerBackend(RobotWorld *world) : 
  ForceFieldBackend(world)
{
}

void PlannerBackend::AddPlannerInput(PlannerInput& _in){
  planner = new HierarchicalMotionPlanner(world, _in);
  in = _in;
  //planner->solve();
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

}
void PlannerBackend::RenderScreen(){
  BaseT::RenderScreen();

  //std::string line;
  //line = "Hierarchy    :\n";
  //uint cur_level = planner->GetSelectedLevel();
  //uint cur_node = planner->GetSelectedNode();
  //uint Nlevels = planner->GetNumberOfLevels();
  //uint Nnodes = planner->GetNumberNodesOnSelectedLevel();
  //line+=(" ["+std::to_string(cur_level)+"/"+std::to_string(Nlevels)+"]");
  //line+=(" ["+std::to_string(cur_node)+"/"+std::to_string(Nnodes)+"]");
  //  uint NumberNodesOnLevel(uint level);

  //DrawText(line_x_pos,line_y_offset,line);
  //line_y_offset += line_y_offset_stepsize;

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
