#include <stdio.h>
//#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
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
#include "gui.h"
#include "info.h"
#include "planner.h"
#include "object.h"
#include "controller.h"
#include "kinodynamic_planner_path_visualizer.h"

int main(int argc,const char** argv) {
  RobotWorld world;
  Info info;
  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel.xml");
  info(&world);

  //############################################################################
  //obtain start and goal config
  //############################################################################

  Robot *robot = world.robots[0];
  Config p_init = robot->q;
  //p_init.setZero();
  p_init[0]=-0.5;
  p_init[1]=-1;
  p_init[2]=-1;
  p_init[3]=2*M_PI-M_PI/8;
  p_init[4]=-M_PI/16;
  p_init[5]=M_PI/16;
  sim.odesim.SetGravity(Vector3(0,0,0));

  world.background = GLColor(1,1,1);

  //############################################################################
  //free space planner
  //############################################################################
  KinodynamicPlannerPathVisualizer debugger(&world, &sim);

  std::vector<KinodynamicMilestonePath> paths = debugger.GetPaths(p_init);
  for(int i = 0; i < paths.size(); i++){
    backend.VisualizePathSweptVolume(paths[i]);
  }

  ////############################################################################
  ////guification
  ////############################################################################

  std::cout << "start GUI" << std::endl;
  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}




