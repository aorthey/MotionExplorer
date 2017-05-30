#include <Interface/SimTestGUI.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <stdio.h>
#include "gui.h"
#include "info.h"

#include "environment_loader.h"
#include "planner/planner_ompl.h"
#include "planner/planner_ompl_humanoid.h"
#include "planner/planner_contact.h"
#include "iksolver_hrp2.h"

int main(int argc,const char** argv) {
  std::string file = "data/humanoid_mountain.xml";
  EnvironmentLoader env = EnvironmentLoader(file.c_str());
  //MotionPlannerOMPLHumanoid planner(env.GetWorldPtr());
  WorldSimulation sim = env.GetBackendPtr()->sim;
  ODERobot *simrobot = sim.odesim.robot(0);
  //Robot *robot = sim.odesim.robot(0).robot;
  simrobot->EnableSelfCollisions(true);


  //PlannerInput pin = env.GetPlannerInput();
  //std::cout << pin << std::endl;
  //Config p_init = pin.q_init;
  //Config p_goal = pin.q_goal;

  //if(planner.solve(p_init, p_goal)){
  //  //util::SetSimulatedRobot(env.GetRobotPtr(), sim, planner.q );
  //  std::vector<Config> keyframes = planner.GetKeyframes();
  //  env.GetBackendPtr()->AddPath(keyframes);
  //}

  ////   std::vector<Config> keyframes = planner.GetKeyframes();
  ////   env.GetBackendPtr()->AddPath(keyframes);
  //// }

  ////env.GetBackendPtr()->SetIKConstraints(ik.GetIKGoalConstraints(), ik.GetIKRobotName());
  ////env.GetBackendPtr()->VisualizeStartGoal(p_init, p_goal);
  //env.GetBackendPtr()->VisualizePlannerTree(planner.GetTree());
  //env.GetBackendPtr()->HidePlannerTree();
  //env.GetBackendPtr()->HideSweptVolumes();
  env.GetBackendPtr()->ShowRobot();
  //env.GetBackendPtr()->HideCoordinateAxes();
  env.GetBackendPtr()->drawRigidObjectsFaces = 1;
  env.GetBackendPtr()->drawRigidObjectsEdges = 0;
  std::cout << "SelfCollisionsEnabled" << std::endl;
  std::cout << simrobot->SelfCollisionsEnabled() << std::endl;

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("robonaut");
  gui.Run();

  return 0;
}

