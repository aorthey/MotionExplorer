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
#include "environment_loader.h"
#include "planner/planner_ompl.h"
//#include "planner/irreducible_projector.h"
#include "planner/irreducible_projector_sentinel.h"
#include "planner/irreducible_projector_snake.h"
#include "planner/irreducible_projector_humanoid.h"

int main(int argc,const char** argv) {
  //std::string file = "data/hrp2_door_complete.xml";
  //std::string path = "data/paths/hrp2_door2.xml";
  std::string file = "data/hrp2_wall_complete.xml";
  std::string path = "data/paths/humanoid_wall_fixed.xml";

  EnvironmentLoader env = EnvironmentLoader(file.c_str());

  std::vector<Config> headPath = env.GetKeyframesFromFile(path.c_str());

  IrreducibleProjectorHRP2 projector(env.GetRobotPtr());
  projector.setRootPath(headPath);
  std::vector<Config> wholeBodyPath = projector.getSubLinkKeyframes();

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


  //TODO
  //for(int i = 0; i < wholeBodyPath.size(); i++){
  //  Config q = wholeBodyPath.at(i);
  //  for(int j = 0; j < q.size(); j++){
  //    if(std::isnan(q(j))) q(j)=0;
  //  }
  //  wholeBodyPath.at(i) = q;
  //}

  ////############################################################################
  ////guification
  ////############################################################################
  env.GetBackendPtr()->ClearPaths();
  //env.GetBackendPtr()->AddPath(wholeBodyPath,GLColor(0.7,0.1,0.9,0.5),2);
  env.GetBackendPtr()->AddPathInterpolate(wholeBodyPath,GLColor(0.7,0.1,0.9,0.5),2);
  env.GetBackendPtr()->VisualizeStartGoal(wholeBodyPath.at(1), wholeBodyPath.at(wholeBodyPath.size()-1));
  env.GetBackendPtr()->ShowSweptVolumes();
  env.GetBackendPtr()->ShowRobot();

  WorldSimulation sim = env.GetBackendPtr()->sim;
  util::SetSimulatedRobot(env.GetRobotPtr(), sim, wholeBodyPath.at(1));

  std::cout << wholeBodyPath.size() << std::endl;
  std::cout << "start GUI" << std::endl;
  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}



