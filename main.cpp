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

#include "Contact/Grasp.h" //class Grasp
#include "Planning/ContactCSpace.h"
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


  IKSolverGraspHuboLeftHandCylinder ikhubo(&world,0);
  ikhubo.solve();
  ikhubo.SetConfigSimulatedRobot(sim);
  ikhubo.visualize();
  ///IKSolverHubo ikhubo(&world);
  ///ikhubo.solve();
  ///ikhubo.SetConfigSimulatedRobot(sim);

  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest2");
  gui.Run();
  /////###########################################################################
  /////Grasping OBJECT
  /////
  /////  ContactCSpace is a description of a specific contact submanifold, where
  /////  certain points are in contact. Sampling this Cspace corresponds to
  /////  sampling IK confs
  /////
  /////  Grasp worldGrasp: we need to define the IK constraints (i.e. which frames
  /////  are in contact at which position. Also the object id is necessary to
  /////  disable collision checking between links in contact and object)
  /////
  /////  Having done that, we can freely sample around the IK space to find a
  /////  feasible configuration. This corresponds to cspace.Sample(Config&) and
  /////  cspace.Isfeasible(Config)
  /////
  /////###########################################################################

  ///Grasp worldGrasp();
  ///int irobot = 0;
  ///WorldPlannerSettings settings;
  ///settings.InitializeDefault(*world);
  ///SmartPointer<SingleRobotCSpace> freeSpace = SingleRobotCSpace(world,irobot,&settings);

  ///// The contact cspace needs to be initiziliazed with a free CSpace, and also
  ///// with the constraints of contacts + which dofs we like to fix. This defines
  ///// the contact submanifold on which we like to sample for IK solutions)
  ///ContactCSpace cspace(*freeSpace);

  ///Config out;
  /////(1) we need to add the constraints of IK here
  ///for(size_t i=0;i<worldGrasp.constraints.size();i++) {
  ///  //Add IKgoals here
  ///  cspace.AddContact(worldGrasp.constraints[i]);
  ///}
  /////(2) which dofs should be fixed
  ///cspace.fixedDofs = worldGrasp.fixedDofs;
  ///cspace.fixedValues = worldGrasp.fixedValues;
  ///int objectID = world->RigidObjectID(iobject);

  /////(3) which robot links can be in contact with the graspable object?
  ///for(size_t i=0;i<worldGrasp.constraints.size();i++) {
  ///  int k=worldGrasp.constraints[i].link;
  ///  cspace.ignoreCollisions.push_back(pair<int,int>(objectID,world.RobotLinkID(irobot,k)));
  ///}
  ///for(size_t i=0;i<worldGrasp.fixedDofs.size();i++) {
  ///  int k=worldGrasp.fixedDofs[i];
  ///  cspace.ignoreCollisions.push_back(pair<int,int>(objectID,world.RobotLinkID(irobot,k)));
  ///}

  /////(4) draw #iters samples from the contact submanifold using IK solver
  ///int iters = 100;
  ///while(iters-- > 0 ){
  ///  cspace.Sample(out);
  ///  if(cspace.IsFeasible(out)){
  ///    std::cout << "found feasible grasp" << std::endl;
  ///    yield;
  ///  }
  ///}
  ///std::cout << "Sampling Contact Submanifold took " << iters << " iterations" << std::endl;

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

