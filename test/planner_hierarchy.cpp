#include "gui_planner.h"
#include "environment_loader.h"
#include "planner/planner.h"

int main(int argc,const char** argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);

  PlannerInput in = env.GetPlannerInput();

  GLUIPlannerGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.AddPlannerInput(in);
  gui.SetWindowTitle("HierarchicalMotionPlanner");

  if(!in.exists) env.GetBackendPtr()->drawRobot=1;

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "GUI Start" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  gui.Run();

  return 0;
}

