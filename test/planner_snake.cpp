#include "util.h"
#include "info.h"
#include "gui.h"
#include "environment_loader.h"
#include "planner/planner_ompl.h"
#include "controller/controller.h"

#include <stdio.h>
#include <ctime>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>

#include <Contact/Grasp.h> //class Grasp
#include <Planning/ContactCSpace.h>
#include <Modeling/MultiPath.h>
#include <Planning/PlannerSettings.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <Modeling/DynamicPath.h>
#include <Modeling/Paths.h>
#include <Control/PathController.h>

int main(int argc,const char** argv) {

  EnvironmentLoader env = EnvironmentLoader::from_args(argc, argv);

  PlannerInput pin = env.GetPlannerInput();

  MotionPlannerOMPL planner(env.GetWorldPtr(), pin);
  planner.solve();

  env.GetBackendPtr()->AddPlannerOutput( planner.GetOutput() );

  env.GetBackendPtr()->ShowRobot();

  //Robot *inner = env.GetWorldPtr()->robots[1];
  //Robot *outer = env.GetWorldPtr()->robots[2];
  //Config q;
  //q = inner->q;
  //q[0]=-Inf;
  //inner->UpdateConfig(q);
  //q = outer->q;
  //q[0]=-Inf;
  //outer->UpdateConfig(q);

  //env.GetBackendPtr()->sim.odesim.DeleteRobot("sphere_inner");
  //env.GetBackendPtr()->sim.odesim.DeleteRobot("sphere_outer");
  //
  //env.GetWorldPtr()->DeleteRobot("sphere_inner");
  //env.GetWorldPtr()->DeleteRobot("sphere_outer");

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("SweptVolumePath");

  //exit(0);
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "GUI Start" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  gui.Run();

  return 0;
}
