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
#include "src/util.h"
#include "src/object.h"
#include "src/iksolver.h"
#include "src/iksolver_hubo.h"
#include "src/iksolvergrasp_hubolefthandcylinder.h"
#include "src/iksolvergrasp_robonaut.h"
#include "src/controller.h"

int main(int argc,const char** argv) {
  RobotWorld world;

  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/hubo_object.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/atlas_object.xml");
  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/robonaut_object.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_pushdoor.xml");


  Info info(&world);
  info.print();


  IKSolverGraspRobonaut ikrobot(&world,0);
  ikrobot.solve();
  //ikrobot.SetConfigSimulatedRobot(sim);

  //transfer information from IKsolver to backend for vis purposes
  backend.SetIKConstraints( ikrobot.GetIKGoalConstraints(), ikrobot.GetIKRobotName() );
  backend.SetIKCollisions( ikrobot.GetIKCollisions() );

  std::cout << std::string(80, '-') << std::endl;

  ///*
  //plan simple path between A and B
  //

  Config a(ikrobot.GetSolutionConfig());
  a.setZero();
  Config b(ikrobot.GetSolutionConfig());

  MilestonePath path;
  Timer timer;

  WorldPlannerSettings settings;
  settings.InitializeDefault(world);
  settings.robotSettings[0].contactEpsilon = 1e-2;
  settings.robotSettings[0].contactIKMaxIters = 100;

  int irobot = 0;
  SingleRobotCSpace freeSpace = SingleRobotCSpace(world,irobot,&settings);
  //ContactCSpace cspace(freeSpace);

  MotionPlannerFactory factory;
  factory.perturbationRadius = 0.5;
  const int PLANNER_MAX_ITERS = 10;
  int iters = PLANNER_MAX_ITERS;
  SmartPointer<MotionPlannerInterface> planner = factory.Create(&freeSpace,a,b);
  while(iters > 0) {
    planner->PlanMore();
    iters--;
    if(planner->IsSolved()) {
      planner->GetSolution(path);
      break;
    }
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << path.Start() << std::endl;
  std::cout << path.End() << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << "Planner converged after " << PLANNER_MAX_ITERS-iters << "/" << PLANNER_MAX_ITERS << " iterations." << std::endl;

 // double dstep = 0.01;
 // Config cur;
 // for(double d = 0; d < 1; d+=dstep){
 //   path.Eval(d,cur);
 //   std::cout << d << " : " << cur << std::endl;
 // }
  //execute it by 
  //*/

  RobotControllerFactory::Register(new RobotControllerInterface(*(world.robots[0])));

  Robot *robot = world.robots[0];
  util::SetSimulatedRobot(robot,sim,b);
  util::SetSimulatedRobot(robot,sim,a);

  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest2");
  gui.Run();

  //Robot *cube = world.GetRobot("free_cube");
  //Config q = cube->q;
  //std::cout << q << std::endl;
  //q[0] = 1.0;
  //q[1] = 1.0;
  //q[2] = 0.5;
  //cube->UpdateConfig(q);
  //ObjectPlacementManager objectified(&world);
  //objectified.spawn();

  return 0;
}

