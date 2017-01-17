#pragma once
#include <Planning/PlannerSettings.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <Modeling/DynamicPath.h>
#include <Modeling/Paths.h>
#include <Modeling/MultiPath.h>
#include <Planning/RobotTimeScaling.h>

class MotionPlanner{
  private:
    RobotWorld *_world;
    int _irobot;
    WorldSimulation *_sim;
    Config _p_init;
    Config _p_goal;
    MultiPath _path;

  public:
    MotionPlanner(RobotWorld *world, WorldSimulation *sim):
      _world(world),_sim(sim)
    {
      _irobot = 0;
    }


    bool solve(Config &p_init, Config &p_goal){
      MilestonePath milestone_path;
      _p_init = p_init;
      _p_goal = p_goal;
      const int PLANNER_MAX_ITERS = 10;

      Timer timer;

      WorldPlannerSettings settings;
      settings.InitializeDefault(*_world);
      settings.robotSettings[0].contactEpsilon = 1e-2;
      settings.robotSettings[0].contactIKMaxIters = 100;

      SingleRobotCSpace freeSpace = SingleRobotCSpace(*_world,_irobot,&settings);
      //ContactCSpace cspace(freeSpace);

      MotionPlannerFactory factory;
      factory.perturbationRadius = 0.5;
      int iters = PLANNER_MAX_ITERS;
      SmartPointer<MotionPlannerInterface> planner = factory.Create(&freeSpace,p_init,p_goal);
      while(iters > 0) {
        planner->PlanMore();
        iters--;
        if(planner->IsSolved()) {
          planner->GetSolution(milestone_path);
          break;
        }
      }
      std::cout << "Planner converged after " << PLANNER_MAX_ITERS-iters << "/" << PLANNER_MAX_ITERS << " iterations." << std::endl;

      double dstep = 0.1;
      Config cur;
      vector<Config> keyframes;
      for(double d = 0; d <= 1; d+=dstep)
      {
        milestone_path.Eval(d,cur);
        keyframes.push_back(cur);
      }
      _path.SetMilestones(keyframes);
      //
      //
      //RobotControllerFactory::Register(new RobotControllerInterface(*(world.robots[0])));
      //double dstep = 0.1;
      //Config cur;
      //static bool firstTime = true;
      //for(double d = 0; d <= 1; d+=dstep)
      //{
      //  _path.Eval(d,cur);
      //  stringstream ss;
      //  ss<<cur;
      //  if(firstTime){
      //    if(!_sim->robotControllers[0]->SendCommand("set_q",ss.str())) {
      //      fprintf(stderr,"set_q command does not work with the robot's controller\n");
      //      std::cout << "FAILURE" << std::endl;
      //      return false;
      //    } 
      //    firstTime=false;
      //  }else {
      //    if(!_sim->robotControllers[0]->SendCommand("append_q",ss.str())) {
      //      fprintf(stderr,"append_q command does not work with the robot's controller\n");
      //      std::cout << "FAILURE" << std::endl;
      //      return false;
      //    }
      //  }
      //}
      //std::cout << "Sending Path to Controller" << std::endl;
      //Robot *robot = _world->robots[_irobot];
      //util::SetSimulatedRobot(robot,*_sim,p_init);
      //robot->UpdateConfig(p_goal);

      //std::cout << "Done Path to Controller" << std::endl;

      return true;
    }

    void TrajectoryToController(){
      //Path discretization resolution -- it controls
      ////how many points N are used in the optimization.
      ////Running time is empirically quadratic in N
      double xtol=0.01;
      ////Output path discretization resolution
      double ttol=0.01;
      Robot *robot = _world->robots[_irobot];
      bool res=GenerateAndTimeOptimizeMultiPath(robot,_path,xtol,ttol);

    }
    bool PathToController(){
      double dstep = 0.1;
      
      static bool firstTime = true;

      Config cur;
      double d=0.8;
      for(double d = 0; d <= 1; d+=dstep)
      {
        _path.Evaluate(d, cur);

        stringstream ss;
        ss<<cur;
        if(firstTime){
          if(!_sim->robotControllers[0]->SendCommand("set_q",ss.str())) {
            fprintf(stderr,"set_q command does not work with the robot's controller\n");
            std::cout << "FAILURE" << std::endl;
            return false;
          } 
          firstTime=false;
        }else {
          if(!_sim->robotControllers[0]->SendCommand("append_q",ss.str())) {
            fprintf(stderr,"append_q command does not work with the robot's controller\n");
            std::cout << "FAILURE" << std::endl;
            return false;
          }
        }
      }

      std::cout << "Sending Path to Controller" << std::endl;
      Robot *robot = _world->robots[_irobot];
      util::SetSimulatedRobot(robot,*_sim,_p_init);
      robot->UpdateConfig(_p_goal);
      std::cout << "Done Path to Controller" << std::endl;
      return true;
    }
    //*/
};

