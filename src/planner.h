#pragma once
#include <Modeling/MultiPath.h>
#include <Planning/PlannerSettings.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <Modeling/DynamicPath.h>
#include <Modeling/Paths.h>

class MotionPlanner{
  private:
    RobotWorld *_world;
    int _irobot;
    WorldSimulation *_sim;
  public:
    MotionPlanner(RobotWorld *world, WorldSimulation *sim):
      _world(world),_sim(sim)
    {
      _irobot = 0;
    }
    bool solve(Config &p_init, Config &p_goal){
      const int PLANNER_MAX_ITERS = 10;

      MilestonePath path;
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
          planner->GetSolution(path);
          break;
        }
      }
      std::cout << "Planner converged after " << PLANNER_MAX_ITERS-iters << "/" << PLANNER_MAX_ITERS << " iterations." << std::endl;

      //RobotControllerFactory::Register(new RobotControllerInterface(*(world.robots[0])));

      double dstep = 0.1;
      Config cur;
      static bool firstTime = true;
      for(double d = 0; d < 1; d+=dstep){
        path.Eval(d,cur);
        stringstream ss;
        ss<<cur;
        if(firstTime){
          if(!_sim->robotControllers[0]->SendCommand("set_q",ss.str())) {
            fprintf(stderr,"set_q command does not work with the robot's controller\n");
            std::cout << "FAILURE" << std::endl;
            return false;
          } 
        }else {
          if(!_sim->robotControllers[0]->SendCommand("append_q",ss.str())) {
            fprintf(stderr,"append_q command does not work with the robot's controller\n");
            std::cout << "FAILURE" << std::endl;
            return false;
          }
        }
      }

      Robot *robot = _world->robots[_irobot];
      util::SetSimulatedRobot(robot,*_sim,p_init);
      robot->SetConfig(p_goal);
      return true;
    }
};

