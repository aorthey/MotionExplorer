#include <Interface/SimTestGUI.h>
#include <stdio.h>
#include "gui.h"
#include "info.h"

int main(int argc,const char** argv) {
  RobotWorld world;
  ForceFieldBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hrp2.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_2.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/hubo_object.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel/sentinel.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel_complete.xml");
  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/snake_corner.xml");
  Info info;
  info(&world);

  GLUIForceFieldGUI gui(&backend,&world);
  gui.SetWindowTitle("DisplayRobot");
  gui.Run();
  return 0;
}

