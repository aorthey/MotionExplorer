#include "gui/gui_planner.h"
#include "environment_loader.h"
#include "planner/planner.h"

int main(int argc, char **argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);

  //std::string replacement_str = std::string(80,'#');
  std::string replacement_str = "quotient";
  strncpy(argv[0], replacement_str.c_str(), strlen(argv[0]));
  for(int i = 1; i < argc; i++) memset(argv[i], 0, strlen(argv[i]));

  PlannerMultiInput in = env.GetPlannerInput();

  GLUIPlannerGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.AddPlannerInput(in);
  gui.SetWindowTitle("MotionPlannerGUI");
  if(in.inputs.empty()) env.GetBackendPtr()->state("draw_robot").active = 1;

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "GUI Start" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  gui.Run();

  return 0;
}
