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
#include "planner/planner_contact.h"
#include "iksolver_hrp2.h"

int main(int argc,const char** argv) {
  std::string file = "data/hrp2_wall.xml";
  EnvironmentLoader env = EnvironmentLoader(file.c_str());
  MotionPlannerContact planner(env.GetWorldPtr());

  WorldSimulation sim = env.GetBackendPtr()->sim;
  //IKSolverHubo ikhubo(env.GetWorldPtr());
  //ikhubo.solve();
  //ikhubo.SetConfigSimulatedRobot(sim);

  PlannerInput pin = env.GetPlannerInput();
  std::cout << pin << std::endl;
  Config p_init = pin.q_init;
  Config p_goal = pin.q_goal;

  if(planner.solve(p_init, p_goal)){
    util::SetSimulatedRobot(env.GetRobotPtr(), sim, planner.q );
  }
  //   std::vector<Config> keyframes = planner.GetKeyframes();
  //   env.GetBackendPtr()->AddPath(keyframes);
  // }

  //env.GetBackendPtr()->SetIKConstraints(ik.GetIKGoalConstraints(), ik.GetIKRobotName());
  env.GetBackendPtr()->VisualizeStartGoal(p_init, p_goal);
  env.GetBackendPtr()->ShowSweptVolumes();
  env.GetBackendPtr()->ShowRobot();
  env.GetBackendPtr()->HideCoordinateAxes();

  GLUIForceFieldGUI gui(env.GetBackendPtr(),env.GetWorldPtr());
  gui.SetWindowTitle("IK");
  gui.Run();

  return 0;
}

