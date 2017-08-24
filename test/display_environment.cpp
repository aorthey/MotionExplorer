#include <Interface/SimTestGUI.h>
#include <stdio.h>
#include "gui.h"
#include "info.h"
#include "util.h"
#include "environment_loader.h"

int main(int argc,const char** argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);

  Info info;
  info(env.GetWorldPtr());
  WorldSimulation sim = env.GetBackendPtr()->sim;
  ODERobot *simrobot = sim.odesim.robot(0);
  //Robot *robot = sim.odesim.robot(0).robot;
  simrobot->EnableSelfCollisions(true);

  //std::cout << env.GetPlannerInput() << std::endl;
  //env.GetBackendPtr()->VisualizeStartGoal(env.GetPlannerInput().q_init, env.GetPlannerInput().q_goal);
  env.GetBackendPtr()->ShowRobot();

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("DisplayEnvironment");
  gui.Run();

  return 0;
}



