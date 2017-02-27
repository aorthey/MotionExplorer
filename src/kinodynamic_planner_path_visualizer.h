#pragma once

class KinodynamicPlannerPathVisualizer{
  public:
    RobotWorld *_world;
    int _irobot;
    WorldSimulation *_sim;

    KinodynamicPlannerPathVisualizer(RobotWorld *world, WorldSimulation *sim):
      _world(world),_sim(sim)
    {
      _irobot = 0;
    }

    std::vector<KinodynamicMilestonePath> GetPaths(Config& p_init){
      Robot *robot = _world->robots[_irobot];
      robot->UpdateConfig(p_init);
      util::SetSimulatedRobot(robot,*_sim,p_init);

      WorldPlannerSettings settings;
      settings.InitializeDefault(*_world);
      SingleRobotCSpace cspace = SingleRobotCSpace(*_world,_irobot,&settings);
      KinodynamicCSpaceSentinelAdaptor kcspace(&cspace);
      std::vector<KinodynamicMilestonePath> pathvec;

      std::vector<double> control;
      control.push_back(-1);
      control.push_back(-0.75);
      control.push_back(-0.25);
      control.push_back(0);
      control.push_back(0.25);
      control.push_back(0.75);
      control.push_back(1);

      int steps=  500;

      int Npaths = control.size();
      for(int yaw = 0; yaw < Npaths; yaw++)
      {
        for(int pitch = 0; pitch < Npaths; pitch++)
        {
          KinodynamicMilestonePath kd_path;
          kd_path.milestones.push_back(p_init);
          ControlInput u;

          for(int j = 0; j < steps; j++){
            u.resize(p_init.size());
            u.setZero();
            //Rotation Control
            u(0) = 0;
            //u(0) = 0;
            u(1) = control[pitch];
            u(2) = control[yaw];

            //Translation Control
            u(3) = 1;
            u(4) = 0;
            u(5) = 0;
            kd_path.Append(u, &kcspace);
          }

          KinodynamicMilestonePath path;
          //double dstep = 0.1;
          //Config cur;
          //vector<Config> keyframes;
          //for(double d = 0; d <= 1; d+=dstep)
          //{
          //  kd_path.Eval(d,cur);
          //  std::cout << cur(3) << std::endl;
          //  //keyframes.push_back(cur);
          //}
          //std::cout << std::string(80, '-') << std::endl;
          //path.SetMilestones(keyframes);
          pathvec.push_back(kd_path);
        }
      }
      return pathvec;

    }

};
