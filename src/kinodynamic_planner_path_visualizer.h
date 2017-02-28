#pragma once

#include <Modeling/World.h>
#include <Simulation/WorldSimulation.h>
#include <KrisLibrary/planning/KinodynamicPath.h>

class KinodynamicPlannerPathVisualizer{
  public:
    RobotWorld *_world;
    int _irobot;
    WorldSimulation *_sim;
    KinodynamicPlannerPathVisualizer(RobotWorld *world, WorldSimulation *sim);
    std::vector<KinodynamicMilestonePath> GetPaths(Config& p_init);
};
