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
#include "info.h"
#include "planner.h"
#include "object.h"
#include "controller.h"
#include "gui.h"

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
  p_init[0]=0;
  p_init[1]=0;
  p_init[2]=1;
  std::cout << p_init << std::endl;

  sim.odesim.SetGravity(Vector3(0,0,0));
  Config p_goal(p_init);
  //p_goal.resize(p_init.size());
  //p_goal.setZero();

  p_goal[0]=2;
  p_goal[1]=-0.5;
  p_goal[2]=1;
  //p_goal[3]=0;
  //p_goal[3]=0;

  world.background = GLColor(1,1,1);

  //############################################################################
  //free space planner
  //############################################################################
  MotionPlanner planner(&world, &sim);

  if(planner.solve(p_init, p_goal,100,false)){
    info(planner.GetPath());
    std::cout << "send to controller" << std::endl;
    //planner.SendToController();
    std::cout << "VisualizePathSweptVolume" << std::endl;
    backend.VisualizePathSweptVolume(planner.GetPath());
  }
  backend.VisualizePlannerTree(planner.GetTree());


  ////############################################################################
  ////guification
  ////############################################################################

  std::cout << "start GUI" << std::endl;
  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}



