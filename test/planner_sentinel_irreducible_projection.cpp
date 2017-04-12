#include <stdio.h>
#include <ctime>

#include <fstream>
#include <sstream>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>

#include <Contact/Grasp.h> //class Grasp
#include <Planning/ContactCSpace.h>
#include <Modeling/MultiPath.h>
#include <Planning/PlannerSettings.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/robotics/Frame.h>
#include <Modeling/DynamicPath.h>
#include <Modeling/Paths.h>
#include <Control/PathController.h>

#include "util.h"
#include "info.h"
#include "controller.h"
#include "gui.h"
#include "planner/planner_ompl.h"
#include "planner/irreducible_projector.h"

int main(int argc,const char** argv) {
  RobotWorld world;
  Info info;
  ForceFieldBackend backend(&world);
  //SimTestBackend backend(&world);
  WorldSimulation& sim=backend.sim;

  sim.odesim.SetGravity(Vector3(0,0,0));

  //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel_complete.xml");
  //backend.Load("kinodynamic_solution_tunnel_environment.xml");

  backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel_complete.xml");
  backend.Load("sentinel_pipe_homotopy_class2.xml");

  //backend.Load("kinodynamic_solution_tunnel_environment_without_torsion.xml");

  //TODO: outsource this part, merge with planner!?
  std::vector<Config> headPath = backend.getKeyFrames();

  Robot *robot = world.robots[0];
  uint N = robot->links.size();
  uint Nsub = N - headPath.at(0).size();
  uint Nhead = headPath.at(0).size();
  uint Nbranches = 8;
  uint Nsubdimension = Nsub/Nbranches;
  uint Nsegments= (Nsubdimension - 2)/3+1;

  assert(Nsub/Nbranches==(int)Nsub/Nbranches);

  std::vector<double> lengths(Nsegments-1);
  double length = 0.19;
  for(int j = 0; j < Nsegments-1; j++){
    lengths.at(j)=length;
  }

  info(&world);

  IrreducibleProjector proj(robot);
  proj.setRootPath(headPath);

  std::vector<Config> wholeBodyPath = proj.getSubLinkKeyframes(lengths, Nbranches);

  //RANDOM VALUES FOR JOINTS

//  std::vector<Config> wholeBodyPath;
//  for(int i = 0; i < headPath.size(); i++){
//    Config q;
//    q.resize(robot->q.size());
//    q.setZero();
//    Config qhead = headPath.at(i);
//    for(int j = 0; j < Nhead; j++){
//      q(j) = qhead(j);
//    }
//
//    uint jctr = Nhead;
//    for(int k = 0; k < Nbranches; k++){
//      q(jctr) = 0;//first link fixed
//      q(jctr+1) = 0;
//
//      for(int j = jctr+2; j < jctr+Nsubdimension; j+=3){
//        double limit = M_PI/16;
//        q(j) = Rand(-limit,limit);
//        q(j+1) = Rand(-limit,limit);
//        q(j+2) =0;
//      }
//      jctr+=Nsubdimension;
//    }
//    wholeBodyPath.push_back(q);
//
//  }


  ////############################################################################
  ////guification
  ////############################################################################
  backend.VisualizePathSweptVolume(wholeBodyPath);
  backend.VisualizeStartGoal(wholeBodyPath.front(),wholeBodyPath.back());
  backend.VisualizePathMilestones(wholeBodyPath, 20);

  //backend.HidePath();
  //backend.HidePlannerTree();
  //backend.HidePlannerStartGoal();

  util::SetSimulatedRobot( robot, sim, wholeBodyPath.front());
  std::cout << "start GUI" << std::endl;
  GLUIForceFieldGUI gui(&backend,&world);
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}



