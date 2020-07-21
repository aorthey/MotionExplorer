#include "gui/gui_planner.h"
#include "environment_loader.h"
#include "planner/planner.h"

int main(int argc, char **argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);

  // env.RenameExec(argc, argv, "MotionPlanningExplorer");

  PlannerMultiInput in = env.GetPlannerInput();

  GLUIPlannerGUI gui(env.GetBackendPtr(), env.GetWorldPtr());
  gui.AddPlannerInput(in);
  gui.SetWindowTitle("MotionPlannerGUI");
  if(in.inputs.empty()) env.GetBackendPtr()->state("draw_robot").active = 1;

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "GUI Start" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  gui.Run();

  return 0;
}
