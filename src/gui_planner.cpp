#include "gui_planner.h"
#include "drawMotionPlanner.h"

PlannerBackend::PlannerBackend(RobotWorld *world) : 
  ForceFieldBackend(world)
{
}

void PlannerBackend::AddPlannerInput(PlannerInput& _in){
  planner = new HierarchicalMotionPlanner(world, _in);
  in = _in;
  planner->solve();
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

  GLDraw::drawPath(selected_path, green, 30);

  GLDraw::drawGLPathStartGoal(robot, qi, qg);

  //const SweptVolume& sv = planner->GetSelectedPathSweptVolume();
  //GLDraw::drawGLPathSweptVolume(robot, sv.GetMatrices(), sv.GetAppearanceStack(), sv.GetColor());

}
void PlannerBackend::RenderScreen(){
  BaseT::RenderScreen();

  std::string line;
  line = "Hierarchy    :\n";
  line+=(" ["+std::to_string(planner->GetCurrentLevel())+"]");

  DrawText(line_x_pos,line_y_offset,line);
  line_y_offset += line_y_offset_stepsize;
  //  uint Nlevels = plannerOutput.at(0).hierarchy.NumberLevels();
  //
  //  const uint Nnodes = plannerOutput.at(0).hierarchy.NumberNodesOnLevel(1);
  //  //const uint selectedNode = hierarchical_level_nodes.at(hierarchical_level);
  //
  //  //root node
  //  line = "";
  //  for(uint k = 0; k < 2*Nnodes; k++) line+="  ";
  //  line+=("<0>");
  //  DrawText(line_x_pos,line_y_offset,line);
  //  line_y_offset += line_y_offset_stepsize;
  //
  //  for(uint i = 1; i < 2; i++){
  //    const uint selectedNode = hierarchical_level_nodes.at(i);
  //
  //    line = "  ";
  //    for(uint k = 0; k < 2*Nnodes; k++) line+=("_");
  //    line+=("|");
  //    for(uint k = 0; k < 2*Nnodes; k++) line+=("_");
  //    DrawText(line_x_pos,line_y_offset,line);
  //    line_y_offset += line_y_offset_stepsize;
  //
  //    line = " ";
  //    for(uint k = 0; k < Nnodes; k++) 
  //      line+=("    |    ");
  //    DrawText(line_x_pos,line_y_offset,line);
  //    line_y_offset += line_y_offset_stepsize;
  //
  //    line = "";
  //    for(uint k = 0; k < Nnodes; k++){
  //      if(k==selectedNode && i<=hierarchical_level)
  //        line+=("<"+std::to_string(k)+">");
  //      else
  //        line+=("  "+std::to_string(k)+" ");
  //    }
  //
  //    DrawText(line_x_pos,line_y_offset,line);
  //    line_y_offset += line_y_offset_stepsize;
  //  }
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
