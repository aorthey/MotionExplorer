#include "gui.h"
#include "environment_loader.h"
#include "planner/planner.h"

int main(int argc,const char** argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);

  PlannerInput pin = env.GetPlannerInput();

  MotionPlanner planner(env.GetWorldPtr(), pin);
  if(planner.solve()){
    env.GetBackendPtr()->AddPlannerOutput( planner.GetOutput() );
  }
  env.GetBackendPtr()->ShowRobot();

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("KlamptSimulator");

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "GUI Start" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  gui.Run();

  return 0;
}
