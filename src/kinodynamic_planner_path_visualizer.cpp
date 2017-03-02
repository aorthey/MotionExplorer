#include "kinodynamic_planner_path_visualizer.h"

#include <Planning/PlannerSettings.h>
#include <Planning/RobotCSpace.h>
#include "util.h"
#include "cspace_sentinel.h"

KinodynamicPlannerPathVisualizer::KinodynamicPlannerPathVisualizer(RobotWorld *world, WorldSimulation *sim):
      _world(world),_sim(sim)
{
  _irobot = 0;
}

std::vector<KinodynamicMilestonePath> KinodynamicPlannerPathVisualizer::GetPaths(Config& p_init){
  Robot *robot = _world->robots[_irobot];
  robot->UpdateConfig(p_init);
  util::SetSimulatedRobot(robot,*_sim,p_init);

  WorldPlannerSettings settings;
  settings.InitializeDefault(*_world);
  SingleRobotCSpace cspace = SingleRobotCSpace(*_world,_irobot,&settings);
  KinodynamicCSpaceSentinelAdaptor kcspace(&cspace);
  std::vector<KinodynamicMilestonePath> pathvec;

  std::vector<double> yawcontrol;
  yawcontrol.push_back(1);
  yawcontrol.push_back(-1);

  std::vector<double> pitchcontrol;
  pitchcontrol.push_back(-1);
  pitchcontrol.push_back(1);

  int steps = 500;

  for(int yaw = 0; yaw < yawcontrol.size(); yaw++)
  {
    std::cout << "CONTROL " << yawcontrol.at(yaw) << std::endl;
    for(int pitch = 0; pitch < pitchcontrol.size(); pitch++)
    {
      KinodynamicMilestonePath kd_path;
      kd_path.milestones.push_back(p_init);
      ControlInput u;

      for(int j = 0; j < steps; j++){
        u.resize(p_init.size());
        u.setZero();
        //Rotation Control
        //u(0) = control.at(pitch);
        //u(0) = 0;
        u(1) = pitchcontrol.at(pitch);
        u(2) = yawcontrol.at(yaw);

        //Translation Control
        u(3) = 1;
        u(4) = 0;
        u(5) = 0;
        kd_path.Append(u, &kcspace);
      }

      KinodynamicMilestonePath path;
      pathvec.push_back(kd_path);
    }
  }
  return pathvec;

}
