#include <Interface/SimTestGUI.h>
#include <stdio.h>
#include "gui.h"
#include "info.h"
#include "util.h"
#include "environment_loader.h"

int main(int argc,const char** argv) {

  //file = "home/aorthey/git/orthoklampt/data/snake_turbine.xml";
  //std::string file = "data/hubo_object.xml";
  //std::string file = "data/wall.xml";
  //std::string file = "data/snake_turbine.xml";
  //std::string file = "data/human.xml";
  std::string file = "data/hrp2_door_complete.xml";
  EnvironmentLoader env = EnvironmentLoader(file.c_str());

  std::cout << env.GetPlannerInput() << std::endl;
  env.GetBackendPtr()->VisualizeStartGoal(env.GetPlannerInput().q_init, env.GetPlannerInput().q_goal);
  env.GetBackendPtr()->ShowRobot();

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("DisplayEnvironment");
  gui.Run();

  return 0;
}



