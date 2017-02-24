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

    std::vector<MultiPath> GetPaths(Config& p_init){
      Robot *robot = _world->robots[_irobot];
      robot->UpdateConfig(p_init);
      util::SetSimulatedRobot(robot,*_sim,p_init);

      WorldPlannerSettings settings;
      settings.InitializeDefault(*_world);
      SingleRobotCSpace cspace = SingleRobotCSpace(*_world,_irobot,&settings);
      KinodynamicCSpaceSentinelAdaptor kcspace(&cspace);
      std::vector<MultiPath> pathvec;

      std::vector<double> control;
      control.push_back(-1);
      control.push_back(-0.5);
      control.push_back(0);
      control.push_back(0.5);
      control.push_back(1);

      int steps=200;

      int Npaths = control.size();
      for(int i = 0; i < Npaths; i++)
      {
        KinodynamicMilestonePath kd_path;
        kd_path.milestones.push_back(p_init);
        ControlInput u;

        for(int j = 0; j < steps; j++){
          u.resize(p_init.size());
          u.setZero();
          //Rotation Control
          //u(1) = control[i];//Rand(-ak,ak);
          u(0) = control[i];

          //Translation Control
          u(3) = 1;
          u(4) = 0;
          u(5) = 0;
          kd_path.Append(u, &kcspace);
        }

        MultiPath path;
        double dstep = 0.01;
        Config cur;
        vector<Config> keyframes;
        for(double d = 0; d <= 1; d+=dstep)
        {
          kd_path.Eval(d,cur);
          for(int i = 3; i < 6; i++){
            if(cur(i)>M_PI){
              cur(i)-=2*M_PI;
            }
          }
          std::cout << d << cur << std::endl;
          keyframes.push_back(cur);
        }
        path.SetMilestones(keyframes);
        pathvec.push_back(path);
      }
      return pathvec;

    }

};
