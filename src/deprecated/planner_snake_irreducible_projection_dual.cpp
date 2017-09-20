#include <stdio.h>
#include <ctime>

#include <fstream>
#include <sstream>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/Timer.h>

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

int main(int argc,const char** argv) {
  std::string file = "data/snake_turbine.xml";

  EnvironmentLoader env = EnvironmentLoader(file.c_str());
  env.LoadPath("data/paths/snake_turbine30.xml");

  std::vector<Config> headPath = env.GetBackendPtr()->getPathKeyFrames(0);

  Robot *robot = env.GetRobotPtr();


  Timer* timer = new Timer();

  IrreducibleProjectorSnake projector(robot);
  projector.setRootPath(headPath);
  std::vector<Config> irreduciblePath = projector.getSubLinkKeyframes();

  double t = timer->ElapsedTime();

  std::cout << "Time for irreducible projection: " << t << std::endl;
  exit(0);

  std::vector<Config> reduciblePath;
  uint N = robot->links.size();
  uint Nhead = headPath.at(0).size();
  uint Nsubdimension = N - Nhead;

  for(int i = 0; i < headPath.size(); i++){
    Config q;
    q.resize(robot->q.size());
    q.setZero();
    Config qhead = headPath.at(i);
    for(int j = 0; j < Nhead; j++){
      q(j) = qhead(j);
    }

    static bool init = true;

    if(init){
      init=false;
    }else{
      const Config qlast = reduciblePath.back();

      for(int j = Nhead; j < Nhead+Nsubdimension; j+=3){
        double dt = 0.2;
        double limit = M_PI/j;
        q(j) = qlast(j) + dt*Rand(-limit,limit);
        q(j+1) = qlast(j+1) + dt*Rand(-limit,limit);
        q(j+2) =0;

        if(q(j) < robot->qMin(j)) q(j)=robot->qMin(j);
        if(q(j) > robot->qMax(j)) q(j)=robot->qMax(j);
        if(q(j+1) < robot->qMin(j+1)) q(j+1)=robot->qMin(j+1);
        if(q(j+1) > robot->qMax(j+1)) q(j+1)=robot->qMax(j+1);
      }
    }
    reduciblePath.push_back(q);

  }


  ////############################################################################
  ////guification
  ////############################################################################
  env.GetBackendPtr()->ClearPaths();
  //env.GetBackendPtr()->AddPath(wholeBodyPath,GLColor(0.7,0.1,0.9,0.5),5);
  env.GetBackendPtr()->AddPath(irreduciblePath,GLColor(0.1,0.8,0.1,0.5));
  env.GetBackendPtr()->AddPath(reduciblePath,GLColor(0.7,0.1,0.9,0.5));

  WorldSimulation& sim=env.GetBackendPtr()->sim;

  util::SetSimulatedRobot( robot, sim, irreduciblePath.back());
  std::cout << "start GUI" << std::endl;
  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("SweptVolumePath");
  gui.Run();

  return 0;
}



