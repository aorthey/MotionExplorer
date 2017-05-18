#include <Interface/SimTestGUI.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <stdio.h>
#include "gui.h"
#include "info.h"
#include "iksolver.h"
#include "iksolver_hubo.h"

#include "environment_loader.h"

int main(int argc,const char** argv) {
  std::string file = "data/hubo_object.xml";
  EnvironmentLoader env = EnvironmentLoader(file.c_str());

  WorldSimulation sim = env.GetBackendPtr()->sim;

  IKSolverHubo ikhubo(env.GetWorldPtr());
  ikhubo.solve();
  ikhubo.SetConfigSimulatedRobot(sim);

  env.GetBackendPtr()->ShowSweptVolumes();
  env.GetBackendPtr()->ShowRobot();
  env.GetBackendPtr()->ShowCoordinateAxes();

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("IK");
  gui.Run();

  return 0;
}

