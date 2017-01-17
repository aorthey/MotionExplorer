#pragma once
#include <Planning/PlannerSettings.h>
#include <Planning/ContactCSpace.h>
#include <Planning/RobotTimeScaling.h>
#include <Simulation/WorldSimulation.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <Modeling/DynamicPath.h>
#include <Modeling/Paths.h>
#include <Modeling/MultiPath.h>

class MotionPlanner{
  private:
    RobotWorld *_world;
    int _irobot;
    int _icontroller;
    WorldSimulation *_sim;
    Config _p_init;
    Config _p_goal;
    MultiPath _path;

  public:
    const MultiPath& GetPath();
    MotionPlanner(RobotWorld *world, WorldSimulation *sim);
    bool solve(Config &p_init, Config &p_goal);
    void SendCommandStringController(string cmd, string arg);
    bool SendToController();
};

