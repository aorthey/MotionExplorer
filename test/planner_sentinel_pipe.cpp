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
#include "controller.h"
#include "gui.h"
#include "planner/planner_ompl.h"

int main(int argc,const char** argv) {
  RobotWorld world;
  Info info;
  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel_pipedreamin_complete.xml");
  world.robots[0]->qMin[0]=-6;
  world.robots[0]->qMin[1]=-6;
  world.robots[0]->qMin[2]=-1;
  world.robots[0]->qMin[3]=-M_PI;
  world.robots[0]->qMin[4]=-M_PI/2;
  world.robots[0]->qMin[5]=-M_PI;

  world.robots[0]->qMax[0]=6;
  world.robots[0]->qMax[1]=6;
  world.robots[0]->qMax[2]=16;
  world.robots[0]->qMax[3]=M_PI;
  world.robots[0]->qMax[4]=M_PI/2;
  world.robots[0]->qMax[5]=M_PI;

  //world.robots[0]->qMin[6]=0;
  //world.robots[0]->qMax[6]=1e-16;
  info(&world);

  //############################################################################
  //obtain start and goal config
  //############################################################################

  Robot *robot = world.robots[0];
  Config p_init = robot->q;

  sim.odesim.SetGravity(Vector3(0,0,0));


  p_init.setZero();
  p_init[0]=-2.5;
  p_init[1]=-1.5;
  p_init[2]=4.2;
  p_init[3]=-M_PI/4;

  //p_goal[0]=2.0;
  //p_goal[1]=0.3;
  //p_goal[2]=1.3;
  Config p_goal;
  p_goal.resize(p_init.size());
  p_goal.setZero();

  //goal in lower pipe
  p_goal[0]=-0.5;
  p_goal[1]=1.0;
  p_goal[2]=0.0;
  p_goal[3]=M_PI/2;

  //////goal in upper pipe
  p_goal[0]=-5.1;
  p_goal[1]=-0.1;
  p_goal[2]=15;
  p_goal[3]=0;
  p_goal[4]=-M_PI/2;
  p_goal[5]=0;

  world.background = GLColor(1,1,1);

  //############################################################################
  //free space planner
  //############################################################################

  MotionPlannerOMPL planner(&world, &sim);

  if(planner.solve(p_init, p_goal)){
    std::vector<Config> keyframes = planner.GetKeyframes();
    backend.VisualizePathSweptVolume(keyframes);

    //void VisualizeFrame( const Vector3 &p, const Vector3 &e1, const Vector3 &e2, const Vector3 &e3, double frameLength=1.0);
  }

  backend.VisualizeStartGoal(p_init, p_goal);
  backend.VisualizePlannerTree(planner.GetTree());
  backend.Save("sentinel_pipe.xml");
  //backend.Load("kinodynamic_solution_tunnel_environment.xml");

  ////############################################################################
  ////guification
  ////############################################################################

  std::cout << "start GUI" << std::endl;
  GLUIForceFieldGUI gui(&backend,&world);
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}



