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

#include "util.h"
#include "info.h"
#include "gui.h"
#include "environment_loader.h"
#include "planner/planner_ompl.h"
#include "controller/controller.h"

int main(int argc,const char** argv) {

  std::string exec = argv[0];
  std::string file;
  std::vector<std::string> all_args;

  if (argc > 1) {
    file = argv[1];
    all_args.assign(argv + 1, argv + argc);
  }else{
    file = "data/snake_turbine.xml";
  }
  std::cout << "Loading file: " << file << std::endl;


  EnvironmentLoader env = EnvironmentLoader(file.c_str());
  MotionPlannerOMPL planner(env.GetWorldPtr());

  PlannerInput pin = env.GetPlannerInput();

  if(planner.solve(pin)){
  }
  env.GetBackendPtr()->AddPlannerIO( planner.GetInput(), planner.GetOutput() );

  env.GetBackendPtr()->HidePlannerTree();
  env.GetBackendPtr()->ShowRobot();

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("SweptVolumePath");

  std::cout << std::string(80, '-') << std::endl;
  std::cout << "GUI Start" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  gui.Run();

  return 0;
}
