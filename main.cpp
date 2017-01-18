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

#include "src/gui.h"
#include "src/info.h"
#include "src/planner.h"
#include "src/object.h"
#include "src/iksolver.h"
#include "src/iksolver_hubo.h"
#include "src/iksolvergrasp_hubolefthandcylinder.h"
#include "src/iksolvergrasp_robonaut.h"
#include "src/controller.h"

int main(int argc,const char** argv) {
  RobotWorld world;
  Info info;
  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/hubo_object.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/atlas_object.xml");
  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/robonaut_object.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_pushdoor.xml");
  info(&world);

  //############################################################################
  //IK solve for some grasp points
  //############################################################################

  IKSolverGraspRobonaut ikrobot(&world,0);
  ikrobot.solve();

  //############################################################################
  //transfer information from IKsolver to backend for vis purposes
  //############################################################################

  backend.SetIKConstraints( ikrobot.GetIKGoalConstraints(), ikrobot.GetIKRobotName() );
  backend.SetIKCollisions( ikrobot.GetIKCollisions() );

  //############################################################################
  //obtain start and goal config
  //############################################################################

  std::cout << std::string(80, '-') << std::endl;
  Config p_start(ikrobot.GetSolutionConfig());
  p_start.setZero();
  Config p_goal = ikrobot.GetSolutionConfig();

  //############################################################################
  //free space planner
  //############################################################################

  MotionPlanner planner(&world, &sim);
  planner.solve(p_start, p_goal);
  info(planner.GetPath());
  planner.SendToController();

  backend.VisualizePathSweptVolume(planner.GetPath());
  //############################################################################
  //guification
  //############################################################################
  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest2");
  gui.Run();

  return 0;
}

