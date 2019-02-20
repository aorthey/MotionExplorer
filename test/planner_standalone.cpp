#include "gui/gui_planner.h"
#include "environment_loader.h"
#include "planner/planner.h"

int main(int argc, char **argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);
  env.RenameExec(argc, argv, "quotient-standalone");

  PlannerMultiInput in = env.GetPlannerInput();

  GLUIPlannerGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.AddPlannerInput(in);
  gui.SetWindowTitle("MotionPlannerVisualizer");
  gui.backend->OnCommand("planner_advance_until_solution","");

  return 0;
}
