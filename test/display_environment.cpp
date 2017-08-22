#include <Interface/SimTestGUI.h>
#include <stdio.h>
#include "gui.h"
#include "info.h"
#include "util.h"
#include "environment_loader.h"

int main(int argc,const char** argv) {

  std::string exec = argv[0];
  std::string file;
  std::vector<std::string> all_args;

  if (argc > 1) {
    file = argv[1];
    all_args.assign(argv + 1, argv + argc);
  }else{
    std::cout << "Usage: <xml world file>" << std::endl;
    exit(0);
  }
  std::cout << "Loading file: " << file << std::endl;
  EnvironmentLoader env = EnvironmentLoader(file.c_str());

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



