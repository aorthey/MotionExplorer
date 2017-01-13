//#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
//#include <Interface/QSimTestGUI.h>
//#include <KlamptQt/qtsimtestgui.h>
//#include "pickandplace.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <stdio.h>
#include "src/gui.h"
#include "src/info.h"
#include "src/object.h"
#include "src/iksolver.h"
#include "src/iksolver_hubo.h"
#include "src/iksolvergrasp_hubolefthandcylinder.h"
#include "src/iksolvergrasp_robonaut.h"

#include <Contact/Grasp.h> //class Grasp
#include <Planning/ContactCSpace.h>
#include <Modeling/MultiPath.h>
#include <Planning/PlannerSettings.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include "Modeling/Paths.h"


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
  ikrobot.SetConfigSimulatedRobot(sim);
  backend.SetIKConstraints( ikrobot.GetIKGoalConstraints(), ikrobot.GetIKRobotName() );
  backend.SetIKCollisions( ikrobot.GetIKCollisions() );


  std::cout << std::string(80, '-') << std::endl;
  //int maxIters = 100;
  //int numRemainingIters = maxIters;
  //MultiPath result;
  //Timer timer;
  //bool Plan(CSpace* space,const Config& a, const Config& b,
    //int& maxIters,MilestonePath& path)

  /*
  Config b = ikrobot.GetSolutionConfig();
  Config a = ikrobot.GetSolutionConfig();
  std::cout << a << std::endl;

  MotionPlannerFactory factory;
  factory.perturbationRadius = 0.5;
  space = SingleRobotCSpace(*world,irobot,&settings);
  ContactCSpace cspace(*env.freeSpace);

  SmartPointer<MotionPlannerInterface> planner = factory.Create(space,a,b);
  while(maxIters > 0) {
    planner->PlanMore();
    maxIters--;
    if(planner->IsSolved()) {
      planner->GetSolution(path);
    }
  }
  //*/

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

