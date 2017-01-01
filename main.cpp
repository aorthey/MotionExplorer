//#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
//#include <Interface/QSimTestGUI.h>
//#include <KlamptQt/qtsimtestgui.h>
//#include "pickandplace.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <stdio.h>
#include "src/gui.h"
#include "src/info.h"
#include "src/object.h"
#include "src/iksolver.h"
#include "src/iksolver_hubo.h"

#include "Contact/Grasp.h" //class Grasp
//#include "Modeling/MultiPath.h"
//#include "Modeling/Paths.h"


int main(int argc,const char** argv) {
  RobotWorld world;

  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");
  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/hubo_object.xml");
  //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_pushdoor.xml");

  Info info(&world);
  info.print();

  IKSolverHubo ikhubo(&world);
  ikhubo.solve();
  ikhubo.SetConfigSimulatedRobot(sim);

  Grasp worldGrasp;
  //TakeObjectWithLeftHand();

  //robot->UpdateConfig(q);
  //robot->UpdateFrames();
  //ViewRobot *vrobot = world.GetRobotView("hubo");
  //vrobot->robot->UpdateConfig(q);
  //vrobot->robot->UpdateFrames();

  //Robot *huborizer = world.GetRobot("hubo");
  //always load objects over backend, not in world! (backend takes care of
  //associating it with frontend gui)

  //Robot *cube = world.GetRobot("free_cube");
  //Config q = cube->q;
  //std::cout << q << std::endl;
  //q[0] = 1.0;
  //q[1] = 1.0;
  //q[2] = 0.5;
  //cube->UpdateConfig(q);
  //ObjectPlacementManager objectified(&world);
  //objectified.spawn();

  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest2");
  gui.Run();

  // while(1) {
  //         //sim.Advance(dt);
  //         //sim.UpdateModel();
  //         cout<<sim.time<<'\t'<<world.robots[0]->q<<endl;
  // }
  ////run the simulation
  //while(sim.time < 5) {
  //        backend.RenderWorld();
  //        if(sim.time >= 2.0 && sim.time-dt < 2.0) {
  //                Config q;
  //                sim.robotControllers[0]->GetCommandedConfig(q);
  //                q[7] -= 1.0;
  //                //LexicalCast is needed to convert config to string
  //                sim.robotControllers[0]->SendCommand("set_q",LexicalCast(q));
  //                //then move link 7 (hip pitch) 1.5 radians down
  //                q[7] += 1.5;
  //                sim.robotControllers[0]->SendCommand("append_q",LexicalCast(q));
  //        }
  //        sim.Advance(dt);
  //        sim.UpdateModel();
  //        cout<<sim.time<<'\t'<<world.robots[0]->q<<endl;
  //        backend.DoStateLogging_LinearPath(0,"test_state.path");
  //}
  return 0;
}

