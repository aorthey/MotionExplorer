#include <stdio.h>
#include <ctime>
//#include <Interface/SimulationGUI.h>
//#include <Interface/SimTestGUI.h>
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
#include "controller.h"
#include "gui.h"
#include "planner/planner_ompl.h"

int main(int argc,const char** argv) {
  RobotWorld world;
  Info info;
  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel.xml");
  world.robots[0]->qMin[0]=-4;
  world.robots[0]->qMin[1]=-4;
  world.robots[0]->qMin[2]=-4;
  world.robots[0]->qMin[3]=0;
  world.robots[0]->qMin[4]=0;
  world.robots[0]->qMin[5]=0;
  world.robots[0]->qMax[0]=4;
  world.robots[0]->qMax[1]=4;
  world.robots[0]->qMax[2]=4;
  world.robots[0]->qMax[3]=2*M_PI;
  world.robots[0]->qMax[4]=2*M_PI;
  world.robots[0]->qMax[5]=2*M_PI;

  world.robots[0]->qMin[6]=0;
  world.robots[0]->qMax[6]=1e-8;
  info(&world);

  //############################################################################
  //obtain start and goal config
  //############################################################################

  Robot *robot = world.robots[0];
  Config p_init = robot->q;
  //p_init.setZero();

  sim.odesim.SetGravity(Vector3(0,0,0));

  Config p_goal;
  p_goal.resize(p_init.size());
  p_goal.setZero();

  //sentinel

  p_init[0]=-1.3;
  p_init[1]=-0.4;
  p_init[2]=2.7;
  p_init[3]=M_PI/4;

  p_goal[0]=2.0;
  p_goal[1]=0.3;
  p_goal[2]=1.3;

  world.background = GLColor(1,1,1);

  //############################################################################
  //free space planner
  //############################################################################

  std::clock_t start = std::clock();

  MotionPlannerOMPL planner(&world, &sim);

  if(planner.solve(p_init, p_goal)){
    backend.VisualizePathSweptVolume(planner.GetKeyframes());
  }

  std::clock_t end = std::clock();
  double duration = ( end - start ) / (double) CLOCKS_PER_SEC;
  std::cout << "Planning Time T=" << duration << std::endl;

  backend.VisualizeStartGoal(p_init, p_goal);
  backend.VisualizePlannerTree(planner.GetTree());
  backend.Save();
  //backend.Load("state_2017_03_15.xml");

  ////############################################################################
  ////guification
  ////############################################################################

  std::cout << "start GUI" << std::endl;
  GLUIForceFieldGUI gui(&backend,&world);
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}



