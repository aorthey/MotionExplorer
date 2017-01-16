//#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
//#include <Interface/QSimTestGUI.h>
//#include <KlamptQt/qtsimtestgui.h>
//#include "pickandplace.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <stdio.h>
#include "src/gui.h"
#include "src/info.h"
#include "src/object.h"
#include "src/iksolver.h"
#include "src/iksolver_hubo.h"


int main(int argc,const char** argv) {
  RobotWorld world;

  ForceFieldBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/hubo_object.xml");

  Info info;
  print(&world);

  IKSolverHubo ikhubo(&world);
  ikhubo.solve();
  ikhubo.SetConfigSimulatedRobot(sim);

  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest2");
  gui.Run();

  return 0;
}

